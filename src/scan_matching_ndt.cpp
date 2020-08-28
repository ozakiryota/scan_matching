#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

class ScanMatchingNDT{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_pose;
		ros::Subscriber _sub_pc;
		/*publisher*/
		ros::Publisher _pub_pose;
		ros::Publisher _pub_pc;
		/*viewer*/
		pcl::visualization::PCLVisualizer _viewer{"scan_matching_ndt"};
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_now {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_map {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_trans {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_map_filtered {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pc_now_filtered {new pcl::PointCloud<pcl::PointXYZ>};
		/*pose*/
		geometry_msgs::PoseStamped _pose_ekf;
		geometry_msgs::PoseStamped _pose_ndt;
		/*flags*/
		bool _got_first_pose = false;
		/*parameters*/
		double _pc_range;
		double _leafsize_source;
		double _leafsize_target;
		double _trans_epsilon;
		double _stepsize;
		double _resolution;
		int _max_iterations;
		bool _erase_out_of_range_pc;
	public:
		ScanMatchingNDT();
		void callbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void initialRegistration(void);
		bool transformation(void);
		void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range);
		void downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize);
		// void approximateDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize);
		void visualization(void);
		void publication(ros::Time stamp);
		Eigen::Quaternionf quatMsgToEigen(geometry_msgs::Quaternion q_msg);
		geometry_msgs::Quaternion quatEigenToMsg(Eigen::Quaternionf q_eigen);
};

ScanMatchingNDT::ScanMatchingNDT()
	:_nhPrivate("~")
{
	std::cout << "--- scan_matching_ndt ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("pc_range", _pc_range, 100.0);
	std::cout << "_pc_range = " << _pc_range << std::endl;
	_nhPrivate.param("leafsize_source", _leafsize_source, 0.1);
	std::cout << "_leafsize_source = " << _leafsize_source << std::endl;
	_nhPrivate.param("leafsize_target", _leafsize_target, 0.1);
	std::cout << "_leafsize_target = " << _leafsize_target << std::endl;
	_nhPrivate.param("trans_epsilon", _trans_epsilon, 1.0e-8);
	std::cout << "_trans_epsilon = " << _trans_epsilon << std::endl;
	_nhPrivate.param("stepsize", _stepsize, 0.1);
	std::cout << "_stepsize = " << _stepsize << std::endl;
	_nhPrivate.param("resolution", _resolution, 0.1);
	std::cout << "_resolution = " << _resolution << std::endl;
	_nhPrivate.param("max_iterations", _max_iterations, 100);
	std::cout << "_max_iterations = " << _max_iterations << std::endl;
	_nhPrivate.param("erase_out_of_range_pc", _erase_out_of_range_pc, false);
	std::cout << "_erase_out_of_range_pc = " << _erase_out_of_range_pc << std::endl;
	/*subscriber*/
	_sub_pose = _nh.subscribe("/ekf/pose", 1, &ScanMatchingNDT::callbackPose, this);
	_sub_pc = _nh.subscribe("/cloud", 1, &ScanMatchingNDT::callbackPC, this);
	/*publisher*/
	_pub_pose = _nh.advertise<geometry_msgs::PoseStamped>("/ndt/pose", 1);
	_pub_pc = _nh.advertise<sensor_msgs::PointCloud2>("/mapped_points", 1);
	/*viewer*/
	_viewer.setBackgroundColor(1, 1, 1);
	_viewer.addCoordinateSystem(0.5, "axis");
	_viewer.setCameraPosition(0.0, 0.0, 80.0, 0.0, 0.0, 0.0);
}

void ScanMatchingNDT::callbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	_pose_ekf = *msg;
	if(!_got_first_pose)	_got_first_pose = true;
}

void ScanMatchingNDT::callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *_pc_now);
	if(_got_first_pose){
		if(_pc_map->points.empty())	initialRegistration();
		else{
			if(transformation())	publication(msg->header.stamp);
		}
		visualization();
	}
}

void ScanMatchingNDT::initialRegistration(void)
{
	/*transform*/
	Eigen::Vector3f offset(
		_pose_ekf.pose.position.x,
		_pose_ekf.pose.position.y,
		_pose_ekf.pose.position.z
	);  
	Eigen::Quaternionf rotation(
		_pose_ekf.pose.orientation.w,
		_pose_ekf.pose.orientation.x,
		_pose_ekf.pose.orientation.y,
		_pose_ekf.pose.orientation.z
	);
	pcl::transformPointCloud(*_pc_now, *_pc_map, offset, rotation);
	// std::cout << "_pc_now->points.size() = " << _pc_now->points.size() << std::endl;
	// std::cout << "_pc_map->points.size() = " << _pc_map->points.size() << std::endl;
}

bool ScanMatchingNDT::transformation(void)
{
	std::cout << "--- transformation ---" << std::endl; 

	double time_start = ros::Time::now().toSec();

	/*initialize*/
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	/*filtering*/
	std::vector<double> range_local{
		-_pc_range,
		_pc_range,
		-_pc_range,
		_pc_range
	};
	std::vector<double> range_global{
		_pose_ekf.pose.position.x - _pc_range,
		_pose_ekf.pose.position.x + _pc_range,
		_pose_ekf.pose.position.y - _pc_range,
		_pose_ekf.pose.position.y + _pc_range
	};
	if(!_erase_out_of_range_pc){
		passThroughFilter(_pc_map, _pc_map_filtered, range_global);
		passThroughFilter(_pc_now, _pc_now_filtered, range_local);
	}
	else{
		passThroughFilter(_pc_map, _pc_map, range_global);
		*_pc_map_filtered = *_pc_map;
		passThroughFilter(_pc_now, _pc_now_filtered, range_local);
	}
	/*downsampling*/
	// std::cout << "before: _pc_map_filtered->points.size() = " << _pc_map_filtered->points.size() << std::endl;
	downsampling(_pc_now_filtered, _leafsize_source);
	std::cout << "downsampling clock [s] = " << ros::Time::now().toSec() - time_start << std::endl;
	/*drop out*/
	if(_pc_now_filtered->points.empty() || _pc_map_filtered->points.empty())	return false;
	/*set parameters*/
	ndt.setTransformationEpsilon(_trans_epsilon);
	ndt.setStepSize(_stepsize);
	ndt.setResolution(_resolution);
	ndt.setMaximumIterations(_max_iterations);
	/*set cloud*/
	ndt.setInputSource(_pc_now_filtered);
	ndt.setInputTarget(_pc_map_filtered);
	std::cout << "_pc_now_filtered->points.size() = " << _pc_now_filtered->points.size() << std::endl;
	std::cout << "_pc_map_filtered->points.size() = " << _pc_map_filtered->points.size() << std::endl;
	/*initial guess*/
	Eigen::Translation3f init_translation(
		(float)_pose_ekf.pose.position.x,
		(float)_pose_ekf.pose.position.y,
		(float)_pose_ekf.pose.position.z
	);
	Eigen::AngleAxisf init_rotation(
		quatMsgToEigen(_pose_ekf.pose.orientation)
	);
	std::cout << "init_translation = ("
		<< init_translation.x() << ", "
		<< init_translation.y() << ", "
		<< init_translation.z() << ")" << std::endl;
	std::cout << "init_rotation : ("
		<< init_rotation.axis()(0) << ", "
		<< init_rotation.axis()(1) << ", "
		<< init_rotation.axis()(2) << "), "
		<< init_rotation.angle() << " [rad]" << std::endl;
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();
	/*drop out*/
	if(_pc_now_filtered->points.size() > _pc_map_filtered->points.size()){
		pcl::transformPointCloud (*_pc_now_filtered, *_pc_now_filtered, init_guess);
		*_pc_map += *_pc_now_filtered;
		return false;
	}
	/*align*/
	std::cout << "aligning ..." << std::endl; 
	ndt.align(*_pc_trans, init_guess);
	std::cout << "DONE" << std::endl; 
	std::cout << "aligning clock [s] = " << ros::Time::now().toSec() - time_start << std::endl;
	/*drop out*/
	if(!ndt.hasConverged())	return false;
	/*print*/
	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged() 
		<< std::endl << " score: " << ndt.getFitnessScore() << std::endl;
	std::cout << "ndt.getFinalTransformation() = " << std::endl << ndt.getFinalTransformation() << std::endl;
	std::cout << "init_guess = " << std::endl << init_guess << std::endl;
	std::cout << "ndt.getFinalNumIteration() = " << ndt.getFinalNumIteration() << std::endl;
	/*input*/
	Eigen::Matrix4f T = ndt.getFinalTransformation();
	Eigen::Matrix3f R = T.block(0, 0, 3, 3);
	Eigen::Quaternionf q_rot(R);
	q_rot.normalize();
	_pose_ndt.pose.position.x = T(0, 3);
	_pose_ndt.pose.position.y = T(1, 3);
	_pose_ndt.pose.position.z = T(2, 3);
	_pose_ndt.pose.orientation = quatEigenToMsg(q_rot);
	/*register*/
	*_pc_map += *_pc_trans;
	std::cout << "before: _pc_map->points.size() = " << _pc_map->points.size() << std::endl;
	downsampling(_pc_map, _leafsize_target);
	std::cout << "after: _pc_map->points.size() = " << _pc_map->points.size() << std::endl;
	std::cout << "transformation clock [s] = " << ros::Time::now().toSec() - time_start << std::endl;

	return true;
}

void ScanMatchingNDT::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_out, std::vector<double> range)
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

void ScanMatchingNDT::downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(pc);
	vg.setLeafSize((float)leafsize, (float)leafsize, (float)leafsize);
	vg.filter(*tmp);
	*pc = *tmp;
}

/* void ScanMatchingNDT::approximateDownsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double leafsize) */
/* { */
/* 	pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg; */
/* 	avg.setInputCloud(pc); */
/* 	avg.setLeafSize((float)leafsize, (float)leafsize, (float)leafsize); */
/* 	avg.filter(*pc); */
/* 	#<{(| std::cout << "avg.getDownsampleAllData() = " << avg.getDownsampleAllData() << std::endl; |)}># */
/* } */

void ScanMatchingNDT::visualization(void)
{
	_viewer.removeAllPointClouds();

	/* _viewer.addPointCloud<pcl::PointXYZ>(_pc_now, "_pc_now"); */
	/* _viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "_pc_now"); */
	/* _viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "_pc_now"); */

	_viewer.addPointCloud<pcl::PointXYZ>(_pc_map, "_pc_map");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "_pc_map");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "_pc_map");

	_viewer.addPointCloud<pcl::PointXYZ>(_pc_trans, "_pc_trans");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "_pc_trans");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "_pc_trans");

	_viewer.addPointCloud<pcl::PointXYZ>(_pc_map_filtered, "_pc_map_filtered");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "_pc_map_filtered");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "_pc_map_filtered");

	_viewer.spinOnce();
}

void ScanMatchingNDT::publication(ros::Time stamp)
{
	/*publish*/
	_pose_ndt.header.stamp = stamp;
	_pose_ndt.header.frame_id = _pose_ekf.header.frame_id;
	_pub_pose.publish(_pose_ndt);
	/*pc*/
	// _pc_map->header.stamp = _pc_now->header.stamp;
	// _pc_map->header.frame_id = _pose_ekf.header.frame_id;
	sensor_msgs::PointCloud2 pc_msg;
	pcl::toROSMsg(*_pc_map, pc_msg);
	pc_msg.header.stamp = stamp;
	pc_msg.header.frame_id = _pose_ekf.header.frame_id;
	_pub_pc.publish(pc_msg);
}

Eigen::Quaternionf ScanMatchingNDT::quatMsgToEigen(geometry_msgs::Quaternion q_msg)
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
geometry_msgs::Quaternion ScanMatchingNDT::quatEigenToMsg(Eigen::Quaternionf q_eigen)
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
	ros::init(argc, argv, "scan_matching_ndt");
	
	ScanMatchingNDT scan_matching_ndt;

	ros::spin();
}
