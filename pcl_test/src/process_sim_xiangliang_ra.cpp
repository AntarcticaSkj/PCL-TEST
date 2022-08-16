#include <iostream>
#include <string>
#include <vector>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

float sz;
bool show = false;

struct PCAResult
{
    Eigen::Vector4f centroid;
    Eigen::Matrix3f eigenvector;
    Eigen::Vector3f eigenvalue;
};

class ProcessSimXiangliang
{
public:

    ProcessSimXiangliang(const pcl::PointCloud<pcl::PointXYZ>::Ptr target, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr origin)
    {
        // 初始化
        gapNum = 1000;
        scale_originToTarget = 10.0;
        ones_y << 0, -1, 0;
        T_target_origin = Eigen::Matrix4f::Identity();
        originCloud = origin;
        targetCloud = target;
        filteredCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        correctedCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        downsamplingCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

        clock_t start, end, start2;
        start = clock();

        if (sz < 0)
            filterAndCorrect(originCloud, correctedCloud, T_target_origin);
        else
        {
            downsampling(originCloud, sz, downsamplingCloud);
            filterAndCorrect(downsamplingCloud, correctedCloud, T_target_origin);
        }


        start2 = clock();
        std::cout << "filterAndCorrect time = " << double(start2-start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）

        densityMatch(correctedCloud);

        end = clock();
        std::cout << "densityMatch time = " << double(end-start2)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）
    }

    void getProcessResult(Eigen::Matrix4f& _T_target_origin, float& _scale, float& _centroid)
    {
        _T_target_origin = T_target_origin;
        _scale = scale_originToTarget;
        _centroid = centroidX;
    }
private:
    // void removeGroud();

    void filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr&);
    void getDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<int>&, float&, float&);
    void showCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr , pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void filterAndCorrect(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr&, Eigen::Matrix4f&);
    void correctPC(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr&, Eigen::Matrix4f&, bool record);
    PCAResult calPCA(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void calRotation(Eigen::Vector3f, Eigen::Vector3f, double&, Eigen::Vector3f&);
    Eigen::Matrix4f RodriguesMatrixTranslation(Eigen::Vector3f, double);

    void calPCAResultCoordinateForDisplay(const PCAResult&, pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void densityMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr, float, pcl::PointCloud<pcl::PointXYZ>::Ptr&);

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr originCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsamplingCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr correctedCloud;


    int gapNum; // 计算密度时的区间数量

    Eigen::Matrix4f T_target_origin; // 原始点云->模板的旋转矩阵

    float scale_originToTarget; // 原始点云 -> 模板的缩放系数
    Eigen::Vector3f ones_y; // 矫正后的长轴方向，默认为[0;1;0]

    // 矫正后点云的质心的x坐标，用于判断短轴上原点（即出发点）到中点的距离
    float centroidX;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_sim_xiangliang_ra");
    ros::NodeHandle nh;

    nh.param<float>("process_sim_xiangliang_ra/sz",sz, -1.0);
    nh.param<bool>("process_sim_xiangliang_ra/shown",show, false);

    std::string targetPath = "/home/sunkejia/MyData/Point_Cloud/xiangliang_pc/xiangliang_corrected.pcd";
    std::string originPath = "/home/sunkejia/MyData/FAST-LIO/xiangliang/scans.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>());//目标点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr originCloud(new pcl::PointCloud<pcl::PointXYZ>());//源点云
    pcl::io::loadPCDFile(targetPath, *targetCloud);
    pcl::io::loadPCDFile(originPath, *originCloud);


    ProcessSimXiangliang test(targetCloud, originCloud);

    Eigen::Matrix4f T_target_origin;
    float scale_originToTarget;
    float centroidtarget;
    test.getProcessResult(T_target_origin, scale_originToTarget, centroidtarget);

    //std::cout << "Result:\n" << T_target_origin << "\n" << scale_originToTarget << "\n" << centroidtarget << "\n";
    return 0;
}

void ProcessSimXiangliang::downsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr in, float size, pcl::PointCloud<pcl::PointXYZ>::Ptr& out)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud(in);
    vg.setLeafSize(size, size, size);
    vg.filter(*out);
}

// 
void ProcessSimXiangliang::densityMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr correctedCloud)
{
    // 密度匹配
    std::vector<int> dens_c;
    float l_min_c, l_max_c;
    dens_c.resize(gapNum + 1);
    getDensity(correctedCloud, dens_c, l_min_c, l_max_c);

    // 寻找密度变大的值
    std::vector<int> peak;
    int avg = correctedCloud->points.size() / dens_c.size();

    // std::cout << "=== Print peak ===" << std::endl;
    // std::cout << "pc_size = " << correctedCloud->points.size() << std::endl;
    // std::cout << "avg = " << avg << std::endl;

    bool peak_sign = false;

    // 80 118 184 287 392 496 600 704 807 912
    for (int i = 0; i < dens_c.size(); ++i)
        if (dens_c[i] > 3 * avg)
        {
            if (peak_sign)
                continue;
            // std::cout << i << " " ;
            peak.emplace_back(i);
            peak_sign = true;
        }
        else
            peak_sign = false;
    // std::cout << '\n';
    // std::cout << "=== Print done ===" << std::endl;
    // 计算scale
    // std::cout << "=== Print scale ===" << std::endl;
    int same_time = 0;
    bool same_flag = false;
    int same_thre = 2;  // 若重复次数 > s_thre，则说明对应实际间隔为2000
    float allow_error = 0.1;

    int i = peak.size() - 1;
    int start = i;
    int end = start;
    int peak_l1 = peak[i] - peak[--i];
    int peak_l2 = 0;
    for (; i > 0;)
    {   
        peak_l2 = peak[i] - peak[--i];
        if (abs(peak_l2 - peak_l1) < allow_error * peak_l1)
        {
            end = i;
            if (++same_time > same_thre)
            {
                same_flag = true;
            }
        }
        else
        {
            if(same_flag)
                break;
            same_time = 0;
            peak_l1 = peak_l2;
            start = end = i;
        }
    }

    if (same_flag)
        scale_originToTarget = 2000.0 / ((peak[start] - peak[end]) / (same_time + 1) * (l_max_c - l_min_c) / gapNum);
    else
        std::cout << "Can not calculate the scale! Need more data!" << std::endl;
    
    // origin:
    // correctPC time = 1.73068s
    // remove head time = 0.525303s
    // remove tail time = 0.457395s
    // transform:
    // 0.0944339  -0.995299 -0.0215105          0
    // 0.995456   0.094139  0.0143374          0
    // -0.012245 -0.0227667   0.999666          0
    //         0          0          0          1
    // filterAndCorrect time = 5.66636s
    // 9 2 6 0.0384189
    // scale_originToTarget:500.555
    // densityMatch time = 0.379646s

 
    // dowmsampling: 0.05f
    // correctPC time = 0.719946s
    // remove head time = 0.215992s
    // remove tail time = 0.202022s
    // transform:
    // 0.0967318   -0.99507 -0.0218607          0
    // 0.995229  0.0964204   0.014878          0
    // -0.0126968 -0.0231956    0.99965          0
    //         0          0          0          1
    // filterAndCorrect time = 2.42642s
    // 9 2 6 0.0388965
    // scale_originToTarget:504.102
    // densityMatch time = 0.169863s



    // std::cout << start << " " << end << " " << same_time << " " << (l_max_c - l_min_c) / gapNum << "\n";
    // std::cout <<"scale_originToTarget:" << scale_originToTarget << "\n" ;
    // std::cout << "=== Print d one ===" << std::endl;
}

void ProcessSimXiangliang::filterAndCorrect(pcl::PointCloud<pcl::PointXYZ>::Ptr origin, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr& correct, Eigen::Matrix4f& transform)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud2(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Matrix4f trans1(Eigen::Matrix4f::Identity()), trans2(Eigen::Matrix4f::Identity()), trans3(Eigen::Matrix4f::Identity());


    clock_t start, end;
    start = clock();
    // 1. PCA粗矫正
    correctPC(origin, tmpCloud1, trans1, false);

    end = clock();
    // std::cout << "correctPC time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）


    // 2. 去除在负长轴上的数据
    start = clock();
    filterCloud(tmpCloud1, tmpCloud2);
    tmpCloud1.reset(new pcl::PointCloud<pcl::PointXYZ>());
    end = clock();
    // std::cout << "remove head time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）

    // 3. PCA精矫正
    correctPC(tmpCloud2, tmpCloud1, trans2, false);

    // 4. 去尾
    start = clock();
    std::vector<int> dens;
    float l_min, l_max;
    dens.resize(gapNum + 1);
    getDensity(tmpCloud1, dens, l_min, l_max);

    tmpCloud2.reset(new pcl::PointCloud<pcl::PointXYZ>());
    float threshold = 0.9 * tmpCloud1->points.size();
    int sum = 0;
    int i = 0;
    for (; i < gapNum; i++)
    {
        sum += dens[i];
        if (sum > threshold)
            break;
    }

    float l_max_thre = (l_max - l_min) / gapNum * i + l_min;
    // std::cout << "l_max_thre = " << l_max_thre << std::endl;

    for (auto&& point : tmpCloud1->points)
    {
        if (point.y < l_max_thre)
            tmpCloud2->points.emplace_back(point);
    }

    end = clock();
    // std::cout << "remove tail time = " << double(end-start)/CLOCKS_PER_SEC << "s" << std::endl;  //输出时间（单位：ｓ）


    // 5. PCA最终矫正
    tmpCloud1.reset(new pcl::PointCloud<pcl::PointXYZ>());
    correctPC(tmpCloud2, correct, trans3, true);

    // std::cout << "trans:" << std::endl;
    // std::cout << trans1 << std::endl;
    // std::cout << trans2 << std::endl;
    // std::cout << trans3 << std::endl;

    transform = trans3 * trans2 * trans1;
    // std::cout << "transform:\n" << transform << std::endl;
}

void ProcessSimXiangliang::filterCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered)
{
    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*cloud, min_p, max_p);
    // std::cout << "min_p = " << min_p << "\n" << "max_p = " << max_p << "\n";

    float L_x = max_p.x - min_p.x;
    float L_y = max_p.y - min_p.y;

    if (L_x > L_y) // x是长轴
    {

        if (max_p.x > abs(min_p.x))
        {
            for (auto&& point : cloud->points)
                if (point.x > 0)
                {
                    filtered->points.emplace_back(point);
                }
        }
        else
        {
            for (auto&& point : cloud->points)
                if (point.x < 0)
                {
                    filtered->points.emplace_back(point);
                }
        }

    }
    else // y是长轴
    {
        if (max_p.y > abs(min_p.y))
        {
            for (auto&& point : cloud->points)
                if (point.y > 0)
                {
                    filtered->points.emplace_back(point);
                }
        }
        else
        {
            for (auto&& point : cloud->points)
                if (point.y < 0)
                {
                    filtered->points.emplace_back(point);
                }
        }

    }

}

//计算从向量a -> 向量b 的旋转角度和旋转轴
void ProcessSimXiangliang::calRotation(Eigen::Vector3f u, Eigen::Vector3f v, double &angle, Eigen::Vector3f &vec)
{
	angle = acos(u.dot(v) / (u.norm()*v.norm()));
	if (angle > M_PI / 2)
	{
		u = -u;
		angle = M_PI - angle;
	}
	float i, j, k;
	i = u(1)*v(2) - u(2)*v(1), j = v(0)*u(2) - v(2)*u(0), k = u(0)*v(1) - u(1)*v(0);
	vec << i, j, k;
	double value = sqrt(i*i + j*j + k*k);
	vec(0) = vec(0) / value;
	vec(1) = vec(1) / value;
	vec(2) = vec(2) / value;
}

// 罗德里格斯法：旋转角度和旋转轴 -> 旋转矩阵
Eigen::Matrix4f ProcessSimXiangliang::RodriguesMatrixTranslation(Eigen::Vector3f n, double angle)
{
	//罗德里格斯公式求旋转矩阵
	Eigen::Matrix4f x_transform(Eigen::Matrix4f::Identity());
	x_transform(0, 0) = cos(angle) + n(0)*n(0)*(1 - cos(angle));
	x_transform(1, 0) = n(2)*sin(angle) + n(0)*n(1)*(1 - cos(angle));
	x_transform(2, 0) = -n(1)*sin(angle) + n(0)*n(2)*(1 - cos(angle));
	x_transform(0, 1) = n(0)*n(1)*(1 - cos(angle)) - n(2)*sin(angle);
	x_transform(1, 1) = cos(angle) + n(1)*n(1)*(1 - cos(angle));
	x_transform(2, 1) = n(0)*sin(angle) + n(1)*n(2)*(1 - cos(angle));
	x_transform(0, 2) = n(1)*sin(angle) + n(0)*n(2)*(1 - cos(angle));
	x_transform(1, 2) = -n(0)*sin(angle) + n(1)*n(2)*(1 - cos(angle));
	x_transform(2, 2) = cos(angle) + n(2)*n(2)*(1 - cos(angle));

	return  x_transform;
}

// 计算PCA特征向量的坐标点，用于显示
void ProcessSimXiangliang::calPCAResultCoordinateForDisplay(
    const PCAResult& res, pcl::PointCloud<pcl::PointXYZ>::Ptr pointsDisp)
{
    pcl::PointXYZ A, B;
    A.x = res.centroid[0];
    A.y = res.centroid[1];
    A.z = res.centroid[2];

    B.x = res.centroid[0] + res.eigenvector.col(2)[0] * 100.0;
    B.y = res.centroid[1] + res.eigenvector.col(2)[1] * 100.0;
    B.z = res.centroid[2] + res.eigenvector.col(2)[2] * 100.0; 
    pointsDisp->points.emplace_back(A);
    pointsDisp->points.emplace_back(B);
    pointsDisp->points.emplace_back(A);
}

// 计算origin点云密度，并筛选长轴上坐标值小于0的点
void ProcessSimXiangliang::getDensity(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& dst,
    float& l_min, float& l_max)
{
    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*cloud, min_p, max_p);
    // std::cout << "\n\nStart cal density:" << "\n";
    // std::cout << "min_p = " << min_p << "\n" << "max_p = " << max_p << "\n";
    float L_x = max_p.x - min_p.x;
    float L_y = max_p.y - min_p.y;

    float gap_x = L_x / gapNum;
    float gap_y = L_y / gapNum;

    if (L_x > L_y) // x是长轴
    {
        l_min = min_p.x;
        l_max = max_p.x;
        for (auto& point : cloud->points)
        {
            ++dst[(int)((point.x - min_p.x)/ gap_x)];
        }
    }
    else // y是长轴
    {
        l_min = min_p.y;
        l_max = max_p.y;
        for (auto& point : cloud->points)
        {
            ++dst[(int)((point.y - min_p.y)/ gap_y)];
        }
    }

    // // 打印密度
    // int print_avg_star = 10; // 平均每行要打印多少个星
    // int base = cloud->points.size() / gapNum / print_avg_star; // 一颗星代表多少个点
    // std::cout << "Point Cloud Size = " << cloud->points.size() << std::endl;
    // std::cout << "Number of parts = " << gapNum << std::endl;
    // std::cout << "Base = " << base << std::endl;
    // std::cout << "=== Print Density ===" << std::endl;
    // for(int i = 0; i < dst.size(); ++i)
    // {
    //     std::string s = "";

    //     for (int j = 1; j < (int)(0.5 + dst[i] / base); ++j)
    //         s += "+";
        
    //     std::cout << i << ":" << s << std::endl;
    // }
    // std::cout << "=== Print Done ===" << std::endl;
}


// 暂时没什么用
void ProcessSimXiangliang::showCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("showCloud"));

    viewer->setBackgroundColor(255,255,255);

    int v1(0);
	viewer->createViewPort (0.0, 0.0, 0.5, 1, v1);
	viewer->addCoordinateSystem(10, 0.0, 0.0, 0.0, v1);
	viewer->setBackgroundColor (255, 255, 255, v1);
	viewer->addPointCloud (cloud1, "cloud1", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud1");

    int v2(0);
    viewer->createViewPort (0.5, 0.0, 1, 1, v2);
    viewer->addCoordinateSystem(10, 0.0, 0.0, 0.0, v2);	
	viewer->setBackgroundColor (255, 255, 255, v2);
	viewer->addPointCloud (cloud2, "cloud2", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud2");
  
    while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


// 将点云长轴矫正到Y轴上
void ProcessSimXiangliang::correctPC(pcl::PointCloud<pcl::PointXYZ>::Ptr origin, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr& translation, Eigen::Matrix4f& transform, bool record = false)
{
    // PCAResult resTarget = calPCA(targetCloud);
    PCAResult resSource = calPCA(origin);

    if (record)
        centroidX = resSource.centroid[0];
    // std::cout << "\n=== Print PCA result===" ;
    // std::cout << "\nTarget.centroid = \n" ;
    // std::cout << resTarget.centroid;
    // std::cout << "\nTarget.eigenvector = \n" ; 
    // std::cout << resTarget.eigenvector;
    // std::cout << "\nTarget.eigenvalue = \n" ; 
    // std::cout << resTarget.eigenvalue << "\n";
    // std::cout << "\n\n";

    // std::cout << "\nSource.centroid = \n" ;
    // std::cout << resSource.centroid;
    // std::cout << "\nSource.eigenvector = \n" ; 
    // std::cout << resSource.eigenvector;
    // std::cout << "\nSource.eigenvalue = \n" ; 
    // std::cout << resSource.eigenvalue << "\n";
    // std::cout << "=== Print Done ===\n";


	Eigen::Vector3f n2;
	double angle2;
	calRotation(resSource.eigenvector.col(2), ones_y, angle2, n2);
	transform = RodriguesMatrixTranslation(n2, angle2);
	pcl::transformPointCloud(*origin, *translation, transform);  //源点云整体旋转，最大主方向对齐y轴


    // if (show)
    // {
    //     pcl::visualization::PCLVisualizer viewer("PCLVisualizer");
    //     viewer.initCameraParameters();

    //     // show
    //     int v1(0);
    //     viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    //     viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v1);
    //     viewer.addText("Cloud before transforming", 10, 10, "v1 test", v1);
    //     viewer.addCoordinateSystem(0.5, v1);
    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_v1(targetCloud, 0, 255, 0);
    //     viewer.addPointCloud<pcl::PointXYZ>(targetCloud, color_v1, "color_v1", v1);
    //     viewer.addPointCloud<pcl::PointXYZ>(origin, "source", v1);

    //     // 显示PCA特征向量
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr coordinate1(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr coordinate2(new pcl::PointCloud<pcl::PointXYZ>);
    //     calPCAResultCoordinateForDisplay(resSource, coordinate1);
    //     calPCAResultCoordinateForDisplay(resTarget, coordinate2);
    //     viewer.addPolygon<pcl::PointXYZ>(coordinate1,0,0,0, "coordinate1", v1);
    //     viewer.addPolygon<pcl::PointXYZ>(coordinate2,0,0,0, "coordinate2", v1);

    //     int v2(0);
    //     viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    //     viewer.setBackgroundColor(128.0 / 255.0, 138.0 / 255.0, 135.0 / 255.0, v2);
    //     viewer.addText("Cloud after transforming", 10, 10, "v2 test", v2);
    //     viewer.addCoordinateSystem(0.5, v2);
    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_v2(targetCloud, 0, 255, 0);
    //     viewer.addPointCloud<pcl::PointXYZ>(targetCloud, color_v2, "color_v2", v2);
    //     viewer.addPointCloud<pcl::PointXYZ>(translation, "translationCloud", v2);

    //     while (!viewer.wasStopped())
    //     {
    //         //在此处可以添加其他处理  
    //         viewer.spinOnce(100);
    //     }
    // }

}

// 计算点云的PCA结果
PCAResult ProcessSimXiangliang::calPCA(pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
	Eigen::Vector4f pcaCentroidtarget;//容量为4的列向量
	pcl::compute3DCentroid(*target, pcaCentroidtarget);//计算目标点云质心
	Eigen::Matrix3f covariance;//创建一个3行3列的矩阵，里面每个元素均为float类型
	pcl::computeCovarianceMatrixNormalized(*target, pcaCentroidtarget, covariance);//计算目标点云协方差矩阵
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);//构造一个计算特定矩阵的类对象
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();//eigenvectors计算特征向量
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();//eigenvalues计算特征值

    PCAResult ans = {pcaCentroidtarget, eigenVectorsPCA, eigenValuesPCA};

    return ans;
}