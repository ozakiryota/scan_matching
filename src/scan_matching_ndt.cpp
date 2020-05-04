#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
/* #include <pcl/filters/approximate_voxel_grid.h> */
#include <pcl/visualization/cloud_viewer.h>

class NDTScanMatching{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_pose;
		/*publish*/
		ros::Publisher pub_pose;
		ros::Publisher pub_pc;
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"ndt_scan_matching"};
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_trans {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map_filtered {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now_filtered {new pcl::PointCloud<pcl::PointXYZ>};
		/*pose*/
		geometry_msgs::PoseStamped pose_ekf;
		geometry_msgs::PoseStamped pose_ndt;
		/*flags*/
		bool first_callback_pose = true;
		/*parameters*/
		double pc_range;
		double leafsize_source;
		double leafsize_target;
		double trans_epsilon;
		double stepsize;
		double resolution;
		int max_iterations;
	public:
		NDTScanMatching();
		// void InitializePose(geometry_msgs::PoseStamped& pose);
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void InitialRegistration(void);
		bool Transformation(void);
		void PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range);
		void Downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize);
		/* void ApproximateDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize); */
		void Visualization(void);
		void Publication(void);
		Eigen::Quaternionf QuatMsgToEigen(geometry_msgs::Quaternion q_msg);
		geometry_msgs::Quaternion QuatEigenToMsg(Eigen::Quaternionf q_eigen);
};

NDTScanMatching::NDTScanMatching()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &NDTScanMatching::CallbackPC, this);
	sub_pose = nh.subscribe("/ekf/pose", 1, &NDTScanMatching::CallbackPose, this);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/ndt/pose", 1);
	pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/mapped_points", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	viewer.setCameraPosition(0.0, 0.0, 80.0, 0.0, 0.0, 0.0);

	nhPrivate.param("pc_range", pc_range, 100.0);
	std::cout << "pc_range = " << pc_range << std::endl;
	nhPrivate.param("leafsize_source", leafsize_source, 0.1);
	std::cout << "leafsize_source = " << leafsize_source << std::endl;
	nhPrivate.param("leafsize_target", leafsize_target, 0.1);
	std::cout << "leafsize_target = " << leafsize_target << std::endl;
	nhPrivate.param("trans_epsilon", trans_epsilon, 1.0e-8);
	std::cout << "trans_epsilon = " << trans_epsilon << std::endl;
	nhPrivate.param("stepsize", stepsize, 0.1);
	std::cout << "stepsize = " << stepsize << std::endl;
	nhPrivate.param("resolution", resolution, 0.1);
	std::cout << "resolution = " << resolution << std::endl;
	nhPrivate.param("max_iterations", max_iterations, 100);
	std::cout << "max_iterations = " << max_iterations << std::endl;
}

void NDTScanMatching::CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	pose_ekf = *msg;
	first_callback_pose = false;
}

void NDTScanMatching::CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *pc_now);
	if(!first_callback_pose){
		if(pc_map->points.empty())	InitialRegistration();
		else{
			if(Transformation())	Publication();
		}
		Visualization();
	}
}

void NDTScanMatching::InitialRegistration(void)
{
	/*transform*/
	Eigen::Vector3f offset(
		pose_ekf.pose.position.x,
		pose_ekf.pose.position.y,
		pose_ekf.pose.position.z
	);  
	Eigen::Quaternionf rotation(
		pose_ekf.pose.orientation.w,
		pose_ekf.pose.orientation.x,
		pose_ekf.pose.orientation.y,
		pose_ekf.pose.orientation.z
	);
	pcl::transformPointCloud(*pc_now, *pc_map, offset, rotation);
	/* std::cout << "pc_now->points.size() = " << pc_now->points.size() << std::endl; */
	/* std::cout << "pc_map->points.size() = " << pc_map->points.size() << std::endl; */
}

bool NDTScanMatching::Transformation(void)
{
	std::cout << "=== Transformation ===" << std::endl; 

	double time_start = ros::Time::now().toSec();

	/*initialize*/
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	/*filtering*/
	std::vector<double> range_local{
		-pc_range,
		pc_range,
		-pc_range,
		pc_range
	};
	std::vector<double> range_global{
		pose_ekf.pose.position.x - pc_range,
		pose_ekf.pose.position.x + pc_range,
		pose_ekf.pose.position.y - pc_range,
		pose_ekf.pose.position.y + pc_range
	};
	PassThroughFilter(pc_map, pc_map_filtered, range_global);
	PassThroughFilter(pc_now, pc_now_filtered, range_local);
	/*downsampling*/
	std::cout << "before: pc_map_filtered->points.size() = " << pc_map_filtered->points.size() << std::endl;
	Downsampling(pc_now_filtered, leafsize_source);
	std::cout << "downsampling clock [s] = " << ros::Time::now().toSec() - time_start << std::endl;
	/*drop out*/
	if(pc_now_filtered->points.empty() || pc_map_filtered->points.empty())	return false;
	/*set parameters*/
	ndt.setTransformationEpsilon(trans_epsilon);
	ndt.setStepSize(stepsize);
	ndt.setResolution(resolution);
	ndt.setMaximumIterations(max_iterations);
	/*set cloud*/
	ndt.setInputSource(pc_now_filtered);
	ndt.setInputTarget(pc_map_filtered);
	std::cout << "pc_now_filtered->points.size() = " << pc_now_filtered->points.size() << std::endl;
	std::cout << "pc_map_filtered->points.size() = " << pc_map_filtered->points.size() << std::endl;
	/*initial guess*/
	Eigen::Translation3f init_translation(
		(float)pose_ekf.pose.position.x,
		(float)pose_ekf.pose.position.y,
		(float)pose_ekf.pose.position.z
	);
	Eigen::AngleAxisf init_rotation(
		QuatMsgToEigen(pose_ekf.pose.orientation)
	);
	std::cout << "init_translation = (" << init_translation.x() << ", " << init_translation.y() << ", " << init_translation.z() << ")" << std::endl; 
	std::cout << "init_rotation : (" << init_rotation.axis()(0) << ", " << init_rotation.axis()(1) << ", " << init_rotation.axis()(2) << "), " << init_rotation.angle() << " [rad]" << std::endl; 
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();
	/*drop out*/
	if(pc_now_filtered->points.size() > pc_map_filtered->points.size()){
		pcl::transformPointCloud (*pc_now_filtered, *pc_now_filtered, init_guess);
		*pc_map += *pc_now_filtered;
		return false;
	}
	/*align*/
	std::cout << "aligning ..." << std::endl; 
	ndt.align(*pc_trans, init_guess);
	std::cout << "DONE" << std::endl; 
	std::cout << "aligning clock [s] = " << ros::Time::now().toSec() - time_start << std::endl;
	/*drop out*/
	if(!ndt.hasConverged())	return false;
	/*print*/
	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged () 
		<< std::endl << " score: " << ndt.getFitnessScore () << std::endl;
	std::cout << "ndt.getFinalTransformation()" << std::endl << ndt.getFinalTransformation() << std::endl;
	std::cout << "init_guess" << std::endl << init_guess << std::endl;
	/*input*/
	Eigen::Matrix4f T = ndt.getFinalTransformation();
	Eigen::Matrix3f R = T.block(0, 0, 3, 3);
	Eigen::Quaternionf q_rot(R);
	q_rot.normalize();
	pose_ndt.pose.position.x = T(0, 3);
	pose_ndt.pose.position.y = T(1, 3);
	pose_ndt.pose.position.z = T(2, 3);
	pose_ndt.pose.orientation = QuatEigenToMsg(q_rot);
	/*register*/
	*pc_map += *pc_trans;
	std::cout << "before: pc_map->points.size() = " << pc_map->points.size() << std::endl;
	Downsampling(pc_map, leafsize_target);
	std::cout << "after: pc_map->points.size() = " << pc_map->points.size() << std::endl;
	std::cout << "transformation clock [s] = " << ros::Time::now().toSec() - time_start << std::endl;

	return true;
}

void NDTScanMatching::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range)
{
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pc_in);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(range[0], range[1]);
	pass.filter(*pc_out);
	pass.setInputCloud(pc_out);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(range[2], range[3]);
	pass.filter(*pc_out);
}

void NDTScanMatching::Downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(pc);
	vg.setLeafSize((float)leafsize, (float)leafsize, (float)leafsize);
	vg.filter(*tmp);
	*pc = *tmp;
}

/* void NDTScanMatching::ApproximateDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize) */
/* { */
/* 	pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg; */
/* 	avg.setInputCloud(pc); */
/* 	avg.setLeafSize((float)leafsize, (float)leafsize, (float)leafsize); */
/* 	avg.filter(*pc); */
/* 	#<{(| std::cout << "avg.getDownsampleAllData() = " << avg.getDownsampleAllData() << std::endl; |)}># */
/* } */

void NDTScanMatching::Visualization(void)
{
	viewer.removeAllPointClouds();

	/* viewer.addPointCloud<pcl::PointXYZ>(pc_now, "pc_now"); */
	/* viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "pc_now"); */
	/* viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "pc_now"); */

	viewer.addPointCloud<pcl::PointXYZ>(pc_map, "pc_map");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "pc_map");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "pc_map");

	viewer.addPointCloud<pcl::PointXYZ>(pc_trans, "pc_trans");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "pc_trans");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "pc_trans");

	viewer.addPointCloud<pcl::PointXYZ>(pc_map_filtered, "pc_map_filtered");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "pc_map_filtered");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "pc_map_filtered");

	viewer.spinOnce();
}

void NDTScanMatching::Publication(void)
{
	/*publish*/
	pose_ndt.header.stamp = pose_ekf.header.stamp;
	pose_ndt.header.frame_id = pose_ekf.header.frame_id;
	pub_pose.publish(pose_ndt);
	/*pc*/
	pc_map->header.stamp = pc_now->header.stamp;
	pc_map->header.frame_id = pose_ekf.header.frame_id;
	sensor_msgs::PointCloud2 pc_pub;
	pcl::toROSMsg(*pc_map, pc_pub);
	pub_pc.publish(pc_pub);	
}

Eigen::Quaternionf NDTScanMatching::QuatMsgToEigen(geometry_msgs::Quaternion q_msg)
{
	Eigen::Quaternionf q_eigen(
		(float)q_msg.w,
		(float)q_msg.x,
		(float)q_msg.y,
		(float)q_msg.z
	);
	q_eigen.normalize();
	return q_eigen;
}
geometry_msgs::Quaternion NDTScanMatching::QuatEigenToMsg(Eigen::Quaternionf q_eigen)
{
	geometry_msgs::Quaternion q_msg;
	q_msg.x = (double)q_eigen.x();
	q_msg.y = (double)q_eigen.y();
	q_msg.z = (double)q_eigen.z();
	q_msg.w = (double)q_eigen.w();
	return q_msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ndt_scan_matching");
	
	NDTScanMatching ndt_scan_matching;

	ros::spin();
}
