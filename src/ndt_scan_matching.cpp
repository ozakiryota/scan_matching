#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
/* #include <tf/tf.h> */
/* #include <pcl/common/transforms.h> */
#include <pcl/registration/ndt.h>
/* #include <pcl/filters/approximate_voxel_grid.h> */
/* #include <eigen_conversions/eigen_msg.h> */
/* #include <pcl/filters/passthrough.h> */
/* #include <tf/transform_broadcaster.h> */
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
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"ndt_scan_matching"};
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_map {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_trans {new pcl::PointCloud<pcl::PointXYZ>};
		/*pose*/
		geometry_msgs::PoseStamped pose_ekf;
		geometry_msgs::PoseStamped pose_ndt;
		/*flags*/
		bool first_callback_pose = true;
		/*parameters*/
		double pc_range;
		float leafsize;
		double trans_epsilon;
		double stepsize;
		double resolution;
		int max_iterations;
	public:
		NDTScanMatching();
		// void InitializePose(geometry_msgs::PoseStamped& pose);
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
		void InitialRegistration(void);
		void Transformation(void);
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
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	viewer.setCameraPosition(0.0, 0.0, 80.0, 0.0, 0.0, 0.0);

	nhPrivate.param("pc_range", pc_range, 100.0);
	std::cout << "pc_range = " << pc_range << std::endl;
	nhPrivate.param("leafsize", leafsize, {0.01f});
	std::cout << "leafsize = " << leafsize << std::endl;
	nhPrivate.param("trans_epsilon", trans_epsilon, 1.0e-8);
	std::cout << "trans_epsilon = " << trans_epsilon << std::endl;
	nhPrivate.param("stepsize", stepsize, 0.1);
	std::cout << "stepsize = " << stepsize << std::endl;
	nhPrivate.param("resolution", resolution, 0.1);
	std::cout << "resolution = " << resolution << std::endl;
	nhPrivate.param("max_iterations", max_iterations, 100);
	std::cout << "max_iterations = " << max_iterations << std::endl;
}

void NDTScanMatching::CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *pc_now);
	if(!first_callback_pose){
		if(pc_map->points.empty())	InitialRegistration();
		else{
			Transformation();
			Publication();
		}
		Visualization();
	}
}

void NDTScanMatching::CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	pose_ekf = *msg;
	first_callback_pose = false;
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

void NDTScanMatching::Transformation(void)
{
	std::cout << "Transformation" << std::endl; 

	/*ndt*/
	std::cout << "test1" << std::endl; 
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	/*set parameters*/
	std::cout << "test2" << std::endl; 
	ndt.setTransformationEpsilon(trans_epsilon);
	ndt.setStepSize(stepsize);
	ndt.setResolution(resolution);
	ndt.setMaximumIterations(max_iterations);
	/*set cloud*/
	std::cout << "test3" << std::endl; 
	ndt.setInputSource(pc_now);
	ndt.setInputTarget(pc_map);
	/*initial guess*/
	std::cout << "test4" << std::endl; 
	Eigen::Translation3f init_translation(
		pose_ekf.pose.position.x,
		pose_ekf.pose.position.y,
		pose_ekf.pose.position.z
	);
	std::cout << "test5" << std::endl; 
	Eigen::AngleAxisf init_rotation(
		QuatMsgToEigen(pose_ekf.pose.orientation)
	);
	std::cout << "test6" << std::endl; 
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();
	/*align*/
	std::cout << "align ..."; 
	ndt.align(*pc_trans, init_guess);
	std::cout << "DONE" << std::endl; 
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
}

void NDTScanMatching::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud<pcl::PointXYZ>(pc_now, "pc_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "pc_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "pc_now");

	viewer.addPointCloud<pcl::PointXYZ>(pc_map, "pc_map");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "pc_map");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "pc_map");
}

void NDTScanMatching::Publication(void)
{
	/*publish*/
	pose_ndt.header.stamp = pose_ekf.header.stamp;
	pose_ndt.header.frame_id = pose_ekf.header.frame_id;
	pub_pose.publish(pose_ndt);
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
