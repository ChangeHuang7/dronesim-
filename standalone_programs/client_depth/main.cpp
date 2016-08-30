
//****************************************************************************************************
//
//Application (client) for sending images from one computer (server) on the network
//and then passing on the image (through file system) to a Matlab app on the same computer
//
//For control of a quadrotor. The other computer runs the Gazebo quad simulation and passes
//images from the quad to this app, which then sends it to the Matlab app running the
//single image depth estimation. Depth maps are then saved as mat files for the computer
//running the LSTM.
//
//Jay Chakravarty
//Aug 2016.
//
//****************************************************************************************************


#include <QCoreApplication>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <string>

#include <sstream>
#include <iomanip>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <netdb.h>



using namespace std;
using namespace cv;




RNG rng( 0xFFFFFFFF );

string path_RGB_image;
string local_machine_root_location = "/usr/data/pchakrav/droneImagesFromAlienware/";
string im_ready_path;
string depth_ready_path;

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

void establishSocketConnection(int &sockfd, int portno, struct sockaddr_in serv_addr, struct hostent *server)
{
    portno = portno;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    //server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    while (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
        cout << "ERROR connecting, retrying...";
        usleep(1e6); //Sleep for 1 second
    }


}

int main(int argc, char *argv[])
{
    path_RGB_image = local_machine_root_location + "image.jpg";
    im_ready_path = local_machine_root_location + "imageReady";
    depth_ready_path = local_machine_root_location + "depthReady";

    // Establish socket client
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    portno = atoi(argv[2]);

    server = gethostbyname(argv[1]);

    cout << "portno: " << portno << endl;

    establishSocketConnection(sockfd, portno, serv_addr, server);


    for(int frame_idx = 0; frame_idx < 10000; ++frame_idx)
    {


        // Receive image from network
        Mat  receivedImg = Mat::zeros( 360,640, CV_8UC3);//
        int  receivedImgSize = receivedImg.total()*receivedImg.elemSize();
        uchar sockData[receivedImgSize];

        cout << "Received image size " << receivedImgSize << endl;

        int bytes = 0;
        for (int i = 0; i < receivedImgSize; i += bytes) {
            bytes = recv(sockfd, sockData +i, receivedImgSize  - i, 0);
            cout << "Bytes: " << bytes << endl;
            if (bytes == 0) {
                establishSocketConnection(sockfd, portno, serv_addr, server);
            //std::cerr("recv failed", 1);
            }
        }

        // Copy image from sockData buffer to opencv image

        int ptr=0;
        for (int i = 0;  i < receivedImg.rows; i++) {
            for (int j = 0; j < receivedImg.cols; j++) {
                //colour image
                receivedImg.at<cv::Vec3b>(i,j) = cv::Vec3b(sockData[ptr+ 0],sockData[ptr+1],sockData[ptr+2]);
                ptr=ptr+3;
                // grey image
//                receivedImg.at<uchar>(i,j) = (sockData[ptr]);
//                ptr=ptr+1;
            }
        }

        imshow("Received", receivedImg);
        waitKey(5);

        // Write received RGB image to file on local machine so that Matlab app
        // running single image depth estimation can process it.

        std::vector<int> compression_params;
          compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
          compression_params.push_back(100);
        cv::imwrite(path_RGB_image, receivedImg, compression_params);

        // Write a file that signifies the jpeg is ready
        FILE * fid = fopen(im_ready_path.c_str(), "w");
        fclose(fid);


        // Check whether depth image is ready and send it across the network
        Mat frame_orig;
        while(!boost::filesystem::exists(depth_ready_path))
        {
            cout << "Depth not ready" << endl;
            usleep(1000);
        }
        if(boost::filesystem::exists(depth_ready_path))
        {
            cout << "Depth ready" << endl;
            frame_orig = imread(local_machine_root_location+"depth.jpg",CV_LOAD_IMAGE_GRAYSCALE);

            Mat frame = Mat::zeros(360,640, CV_8UC1);
            resize(frame_orig, frame, cv::Size(640,360));

            imshow( "depthImageSent", frame );


            frame = (frame.reshape(0,1)); // to make it continuous

            int  imgSize = frame.total()*frame.elemSize();

            // Comment this part if you don't want to send the depth map back to the server
            // computer (Alienware laptop), and you're send the depthmap as a mat file
            // through the file system to Qayd running the LSTM.
            n = write(sockfd,frame.data,imgSize);
            if (n < 0)
                 error("ERROR writing to socket");

            boost::filesystem::remove(depth_ready_path);


        }

    }

    close(sockfd);


    return 0;
}


