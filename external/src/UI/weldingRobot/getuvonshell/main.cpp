#include "getsbuva.h"
#include "histimage.h"
#include<boost/filesystem.hpp>

using namespace boost;
using namespace std;


int main(int argc, char** argv)
{
    string picDir;
    int tc;
    int ti;
    int ns;
    LINE_TYPE_2 t;
    if(argc == 5)
    {
        picDir= argv[1];
        ns= atoi(argv[2]);
        ti= atoi(argv[3]);
        tc= atoi(argv[4]);
    }
    else if(argc == 1)
    {
        picDir= "/mnt/hgfs/share/hk/shell_31_2020_02_20_15_01_05ng/";
                "/mnt/hgfs/share/hk/20200217/";
                "/mnt/hgfs/share/hk/picture_welding/";
                "/mnt/hgfs/share/hk/pic2/";
                "/mnt/hgfs/share/hk/C3/";
                "/mnt/hgfs/share/hk/blade_seam2_1_0_2019_12_19_18_15_26v/";

        if(picDir.substr(picDir.size()-3,2) == "C1")
        {
            cout << "C1 type!" <<  " " << picDir.substr(0,5) << endl;
            ti= 1;
        }
        else if(picDir.substr(picDir.size()-3,2) == "C2")
        {
            cout << "C2 type!" << endl;
            ti= 2;
        }
        else if(picDir.substr(picDir.size()-3,2) == "C3")
        {
            cout << "C3 type!" << endl;
            ti= 3;
        }
        else if(picDir.substr(picDir.size()-3,2) == "ng")
        {
            cout << "VW type!" << endl;
            ti= 4;
        }

        else
        {
            //cout << "wrong type!" << endl;
            //return -1;
            cout << "S type!" << endl;
            ti= 5;
        }
    }
    if(ti == 0) t= LINE_TYPE_2::LT_D;
    else if(ti == 1) t= LINE_TYPE_2::LT_ONE_Z;
    else if(ti == 2) t= LINE_TYPE_2::LT_TWO_Z;
    else if(ti == 3) t= LINE_TYPE_2::LT_J;
    else if(ti == 4) t= LINE_TYPE_2::LT_VW;
    else if(ti == 5) t= LINE_TYPE_2::LT_S;

    int n0= 0,n1= 2920;
    if(boost::filesystem::exists(picDir))
    {
        boost::filesystem::path path(picDir);
        cout << path.filename() << endl;
        cout << path.relative_path() << endl;
        boost::filesystem::directory_iterator end;
        int num= -1;
        for(boost::filesystem::directory_iterator it(path); it != end; it++) ///png jpg
        {
            string temp= it->path().string(); //cout << temp << endl;
            int s= temp.find_last_of("/i_")+1;
            int tn;
            if(temp.find(".png") != -1) tn= stoi(temp.substr(s,temp.find_last_of(".png")-s));
            else if(temp.find(".jpg") != -1) tn= stoi(temp.substr(s,temp.find_last_of(".jpg")-s));
            else continue;
            if(tn > num)
            {
                num = tn;
                cout << num << " ";
            }
        }
        n1= num+1;
//        path /= "Debug";
//        cout << path.string() << endl;
//        cout << filesystem::current_path() << endl;
    }
    cout << n0 << ";" << n1 << " t= " << ti << endl;

    //GetSBuvA *test;

    //GetSBuvJ *test= new GetSBuvJ(t);
    //GetSBuvS *test= new GetSBuvS(t);
    GetSBuvVW *test= new GetSBuvVW(t);
    test->set_weld_off();
    cv::Mat simg,img,grad_y,kernel= GetKerLoG(8,128)*1000000;
    cout << kernel << endl;
    //cout << kernel.at<double>(4,4) << " " << kernel.at<double>(1,4) << " " << kernel.at<double>(7,4) << " " <<  kernel.at<double>(4,4) / kernel.at<double>(0,4) <<  endl;
    //cout << kernel.at<double>(0,4) << " " << kernel.at<double>(1,4) << " " << kernel.at<double>(2,4) << " " <<  kernel.at<double>(3,4) << " " << kernel.at<double>(4,4) <<  endl;
    //cv::imwrite("bigke.png",kernel);
    fstream timeLog;
    timeLog.open("timelog.txt",ios::out);
    for(int i= n0;i < n1;i++)
    {
        simg= cv::imread(picDir+to_string(i)+".png");
        if(!simg.data)
        {
            cout << "wrong Mat!" << endl;
            continue;
        }
        clock_t s= clock();
        test->AddImg(simg);
        timeLog << clock()-s << " ";
        cout << clock()-s << endl;

//        bgr2gray(simg,img,2);
//        cv::filter2D(img,grad_y,CV_16S,kernel);
//        //cv::threshold(grad_y,grad_y,288,255,8);
//        cv::imwrite(to_string(i)+"log.tiff",grad_y);
//        //cv::medianBlur(img,simg,5);
//        cv::Sobel(img,grad_y,CV_16S,1,1,7);
//        //cv::Sobel(grad_y,grad_y,CV_16S,0,1,3);
//        cv::threshold(grad_y,grad_y,-1052,255,1);
//        cv::imwrite(to_string(i)+"sy.jpg",grad_y);

//        vector<short> mv;
//        cv::reduce(grad_y,mv,0,cv::REDUCE_MAX);
//        for(auto &m:mv) cout << m << " ";
//        cout << endl;
        //cv::imwrite(to_string(i)+"r-.jpg",img);
        //cv::threshold(img,img,22,255,8);
        //cv::imwrite("t/"+to_string(i)+"r.jpg",img);
    }
    timeLog.close();
    map<int,cv::Point2d> uv;
    test->confirm(n1-n0);cout << "OK!";
    test->getuv(uv);
    for(auto &u:uv) cout << "uv: " << u.first << " " << (u.second).x << " " << (u.second).y << endl;



//    cv::namedWindow( "s", CV_WINDOW_AUTOSIZE );
//    cv::imshow("s",simg);
//    cv::waitKey(0);
//    //walgo::bgr2gray(simg,img);
//    cv::cvtColor(simg,img,cv::COLOR_BGR2GRAY);
//    cv::namedWindow( "i", CV_WINDOW_AUTOSIZE );
//    cv::imshow("i",img);
//    cv::waitKey(0);

//    walgo::bgr2gray(simg,img,0);
//    cv::namedWindow( "0", CV_WINDOW_AUTOSIZE );
//    cv::imshow("0",img);
//    cv::waitKey(0);

//    walgo::bgr2gray(simg,img,1);
//    cv::namedWindow( "1", CV_WINDOW_AUTOSIZE );
//    cv::imshow("1",img);
//    cv::waitKey(0);

//    walgo::bgr2gray(simg,img,2);
//    cv::namedWindow( "2", CV_WINDOW_AUTOSIZE );
//    cv::imshow("2",img);
//    cv::waitKey(0);

    return 0;
}
