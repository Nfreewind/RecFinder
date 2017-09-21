#define HAVE_OPENCV_FLANN
#define HAVE_OPENCV_IMGPROC
#define HAVE_OPENCV_VIDEO
#define HAVE_OPENCV_OBJDETECT
#define HAVE_OPENCV_CALIB3D
#define HAVE_OPENCV_ML
#define HAVE_OPENCV_HIGHGUI
#define HAVE_OPENCV_CONTRIB


#include<opencv2/opencv.hpp>

#include<iostream>

#include<stdio.h>
#include<sys/time.h>
#include<unistd.h>

using namespace std;
using namespace cv;



class Line
{
public:

	Line(double k_, double b_){ k=k_;b=b_;}

	double k, b;  // lean rate (slope), cut distance(intercept).

	

	void drawLine(Mat& img)
	{
					
	}	


	void calStartEnd()
	{
		
	}


	Point2d startp;
	Point2d endp;
	int img_rows;
	int img_cols;
	
};

typedef struct Segment_
{
public:
	int startpos;
	int length;
	//int wb; // white 1; black 0;
}Segment;



int ratio=4;

typedef struct Triple_
{
	int left;
	int middle;
	int right;
}Triple;

typedef struct Triple_Point_
{
	Point2d left;
	Point2d middle;
	Point2d right;
}Triple_Point;

typedef struct Cluster_
{
	double start;
	double length;
}Cluster;


bool checkAnyBlackPointInCircleR(Point center, Mat& M, int step)
{
	int num=0;
	Point pointCheck(center.x,center.y+step);
	if(pointCheck.y< M.rows && pointCheck.x<M.cols)
		if(M.at<uchar>(pointCheck.y, pointCheck.x)==0)
			num++;

	pointCheck=Point(center.x+step,center.y);
	if(pointCheck.y< M.rows && pointCheck.x<M.cols)
		if(M.at<uchar>(pointCheck.y, pointCheck.x)==0)
			num++;

	pointCheck=Point(center.x-step,center.y);
	if(pointCheck.y< M.rows && pointCheck.x<M.cols)
		if(M.at<uchar>(pointCheck.y, pointCheck.x)==0)
			num++;

	pointCheck=Point(center.x,center.y-step);
	if(pointCheck.y< M.rows && pointCheck.x<M.cols)
		if(M.at<uchar>(pointCheck.y, pointCheck.x)==0)
			num++;

	pointCheck=Point(center.x-step,center.y-step);
	if(pointCheck.y< M.rows && pointCheck.x<M.cols)
		if(M.at<uchar>(pointCheck.y, pointCheck.x)==0)
			num++;
	

	pointCheck=Point(center.x+step,center.y+step);
	if(pointCheck.y< M.rows && pointCheck.x<M.cols)
		if(M.at<uchar>(pointCheck.y, pointCheck.x)==0)
			num++;

	pointCheck=Point(center.x-step,center.y+step);
	if(pointCheck.y< M.rows && pointCheck.x<M.cols)
		if(M.at<uchar>(pointCheck.y, pointCheck.x)==0)
			num++;
	

	pointCheck=Point(center.x+step,center.y-step);
	if(pointCheck.y< M.rows && pointCheck.x<M.cols)
		if(M.at<uchar>(pointCheck.y, pointCheck.x)==0)
			num++;

	
	if(num > 5)
		return 1;
	
	else
		return 0;

}





int minimumMeanlen=0;


bool checkValidLine_longitude(vector<Segment>& vS, Triple& ret, Mat& img, int row, int col) // 1:2:1
{
	for (int i=0;i<vS.size();i++)
	if((i+1<vS.size()) &&(i+2<vS.size()) )	
	{


//cout<<"iam here"<<endl;
//cout<<"vS.size()="<<vS.size()<<endl;
		double deltaLength_lm= (double)( abs ( vS.at(i+1).length - 2* vS.at(i).length ));
		double deltaLength_rm=(double)(abs ( vS.at(i+1).length - 2* vS.at(i+2).length ));
		double meanLength=(double)( ( vS.at(i).length + vS.at(i+1).length +vS.at(i+2).length)/4 );
		int deltaStart_lm= abs(vS.at(i+1).startpos - vS.at(i).startpos); //
		int deltaStart_rm= abs(vS.at(i+1).startpos - vS.at(i+2).startpos); //

		double lengthEqualLimit= (double)(meanLength/3.0) ; //unit:pixels
		double ratioCheckLimit= (double)(meanLength/3.0);  //unit:pixels

//cout<<"deltaLength_lm="<<deltaLength_lm<<endl;
//cout<<"meanLength="<<meanLength<<endl;

	/*	if(   ( deltaLength_lm < lengthEqualLimit)  && ( deltaLength_rm < lengthEqualLimit) && (  abs(2*(double)(meanLength) - (double)(deltaStart_lm)) < lengthEqualLimit )  && (  abs( 2* (double)(meanLength) - (double)(deltaStart_rm)) < lengthEqualLimit )&&  ( meanLength> minimumMeanlen)   )  /// line check // big bug here is 2* needed */
		if( ( deltaLength_lm < lengthEqualLimit)  && ( deltaLength_rm < lengthEqualLimit)&& (  abs(2*(double)(meanLength) - (double)(deltaStart_lm)) < lengthEqualLimit )  && (  abs( 3* (double)(meanLength) - (double)(deltaStart_rm)) < lengthEqualLimit )&&( meanLength> minimumMeanlen)  )  // 3* not 2* big bug // 
		{		


			int drift=meanLength/2;
			int step=meanLength/2;

			Point firststart(vS.at(i).startpos-drift, row);

			
			
			if(checkAnyBlackPointInCircleR(firststart, img, step) ==0    )//add first in startpos and  last out pos constraints
	
			{
				cout<<"one valid doubleline triple detected!"<<endl;
			
			 	ret.left=i;
				ret.middle=i+1;
				ret.right=i+2;
				//cout<<"num. i=" <<i << "  and num. j=" <<j<<" are a good pair"<<endl; 
				cout<<"meanLength="<< meanLength<<endl;

				cout<<"ret.left="<<ret.left<<"   "<<"ret.right="<<ret.right<<endl;
				return 1;

			}
			
			
		}	
			
	}

	return 0;
}


//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&   Vertical Scan  Check &&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&

bool checkValidLine_vertical(vector<Segment>& vS, Triple& ret, Mat& img, int row, int col) // 1:2:1
{
	for (int i=0;i<vS.size();i++)
	if((i+1<vS.size()) &&(i+2<vS.size()) )	
	{


//cout<<"iam here"<<endl;
//cout<<"vS.size()="<<vS.size()<<endl;
		double deltaLength_lm= (double)( abs ( vS.at(i+1).length - 2* vS.at(i).length ));
		double deltaLength_rm=(double)(abs ( vS.at(i+1).length - 2* vS.at(i+2).length ));
		double meanLength=(double)( ( vS.at(i).length + vS.at(i+1).length +vS.at(i+2).length)/4 );
		int deltaStart_lm= abs(vS.at(i+1).startpos - vS.at(i).startpos); //
		int deltaStart_rm= abs(vS.at(i+1).startpos - vS.at(i+2).startpos); //

		double lengthEqualLimit= (double)(meanLength/3.0); //unit:pixels
		double ratioCheckLimit= (double)(meanLength/3.0);  //unit:pixels

//cout<<"deltaLength_lm="<<deltaLength_lm<<endl;
//cout<<"meanLength="<<meanLength<<endl;

	/*	if(   ( deltaLength_lm < lengthEqualLimit)  && ( deltaLength_rm < lengthEqualLimit) && (  abs(2*(double)(meanLength) - (double)(deltaStart_lm)) < lengthEqualLimit )  && (  abs( 2* (double)(meanLength) - (double)(deltaStart_rm)) < lengthEqualLimit )&&  ( meanLength> minimumMeanlen)   )  /// line check // big bug here is 2* needed */
		if( ( deltaLength_lm < lengthEqualLimit)  && ( deltaLength_rm < lengthEqualLimit)&& (  abs(2*(double)(meanLength) - (double)(deltaStart_lm)) < lengthEqualLimit )  && (  abs( 3* (double)(meanLength) - (double)(deltaStart_rm)) < lengthEqualLimit )&&( meanLength> minimumMeanlen)  )  // 3* not 2* big bug // 
		{		


			int drift=meanLength;
			int step=meanLength;

			Point firststart(col, vS.at(i).startpos-drift);

			
			
			if( checkAnyBlackPointInCircleR(firststart, img, step) ==0    )//add first in startpos and  last out pos constraints
	
			{
				cout<<"one valid doubleline triple detected!"<<endl;
			
			 	ret.left=i;
				ret.middle=i+1;
				ret.right=i+2;
				//cout<<"num. i=" <<i << "  and num. j=" <<j<<" are a good pair"<<endl; 
				cout<<"meanLength="<< meanLength<<endl;

				cout<<"ret.left="<<ret.left<<"   "<<"ret.right="<<ret.right<<endl;
				return 1;

			}
			
			
		}	
			
	}

	return 0;
}

//==========================================================================================
//===============================Longitude Scan=======================================================
//==========================================================================================




void longitudeScan(Mat& bina)
{
	int thickness=1;
	int linetype=8;


cout<<"-------------------------------Longitude Scan-----------------------------------"<<endl;

											int cur=0;;

											int startposition=0;

											int flag_first=1;

											Segment temp;

											vector<Segment>vSeg;
											vector<Point2d>vMid;
										
//cout<<raw.rows<<endl;
//cout<<raw.cols<<endl;

											int successnum=0;

											vector<Triple> vTri;
											vector<Triple_Point>vTri_Point;
											Triple_Point tri_success={};
						
int i;
int j;

							for ( i=0;i<bina.rows;i+=1)
							{
												
												int length=0;	
												
												int last=255;

												vSeg.clear(); // clear up all 

												//cout<<endl<<endl;
												//cout<<"============================================================================"<<endl;
												//cout<<"longitude scan in row="<<i<<endl;
			
												for( j=0;j<bina.cols;j++)
													{
														if(bina.at<uchar>(i,j)==0)
															
														{
																if(last==255)// ==1 true first comes here; ==0 not the first time

																{

																	length=0;// clear at beginning
																	startposition=j;

																	//cout<<"j="<<j<<endl;
								
																}	
																length++;
																last=0;

														}

														else

														{
															if(last==0 && (j+10)<bina.cols )
															{
																if(bina.at<uchar>(i,j+1)==255 && bina.at<uchar>(i,j+2)==255 && bina.at<uchar>(i,j+3)==255&& bina.at<uchar>(i,j+4)==255&& bina.at<uchar>(i,j+5)==255 ) // avoid poit white end so check next 5 points
																{
																	temp.startpos=startposition;
																	temp.length=length;

		

																	//length=0;  // big bug of clear here

																	vSeg.push_back(temp);
																}

															}
																
																last=255;


														}	

														

													  }	

	
										if(!vSeg.empty())
										{
//cout<<" vSeg not empty!"<<endl;
												Triple success={0,0,0};
												bool gotit=checkValidLine_longitude(vSeg, success,bina, i,j);

												if(gotit)
												{
													Point2d middle(( vSeg.at(success.right).startpos + vSeg.at(success.left).startpos + vSeg.at(success.left).length)/2 , i ) ;
													vMid.push_back(middle);
													
													Point2d tri_start=Point2d(vSeg.at(success.left).startpos,i);
													Point2d tri_mid=Point2d(vSeg.at(success.middle).startpos,i);
													Point2d tri_last=Point2d(vSeg.at(success.right).startpos,i);

													tri_success={tri_start,tri_mid, tri_last};
												
													vTri_Point.push_back(tri_success);  ////

													cout<<"Left="<<vSeg.at(success.left).startpos<<"     right="<<vSeg.at(success.right).startpos<<endl;
													cout<<"Middle="<< middle<<endl;
													cout<<"row="<<i<<endl;
													cout<<"-------"<<endl;
													successnum++;

												
												}

										}




							}

			if(vMid.size()>0)
			{		
											
cout<<"success num="<<successnum<<endl;
cout<<"all middle are---------"<<endl;		

for (int iter=0;iter<vMid.size();iter++)
	cout<<vMid.at(iter)<<endl;

cout<<endl;


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//cluster


vector<Point2d>vMid_cluster;
double dist[vMid.size()-1];

double min_dist=100000;

for(int ii=0;ii<vMid.size()-1;ii++)
	{
		dist[ii]=vMid.at(ii+1).y- vMid.at(ii).y;
		if(dist[ii]<min_dist)
			min_dist=dist[ii];
	}

Cluster temp_cluster={0,0}; 
Cluster biggest_cluster={0,0};

double cluster_threshold=3; // at least 8 lines bundle

vector<Cluster>vClu;


int fflag_first=1;
for(int ii=0;ii<vMid.size()-1;ii++)
	{
		if(dist[ii]<(min_dist+5))
		{
			if(temp_cluster.length ==0)
			{
				temp_cluster.start=ii;
			}
			
			temp_cluster.length++;	

			if(ii==vMid.size()-1-1)
				if(temp_cluster.length > cluster_threshold)
						{
							vClu.push_back(temp_cluster);
						}	
		}

		else
			if(temp_cluster.length>0)
				{	
					if(temp_cluster.length > cluster_threshold)
						{
							vClu.push_back(temp_cluster);
						}

					temp_cluster.length=0;
				}
				
			
	}







for (int jj=0;jj<vClu.size();jj++)
	for(int ii=vClu.at(jj).start;ii< vClu.at(jj).start + vClu.at(jj).length;ii++)
	{
		
				vMid_cluster.push_back(vMid.at(ii));
line(bina,Point(0,vMid_cluster.back().y), Point( bina.cols,vMid_cluster.back().y), Scalar(0,255,0), thickness, linetype);
			
	}


//cluster end

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

																
//line(bina,vMid_cluster.at(0), vMid_cluster.at(vMid_cluster.size()-1), Scalar(255,255,255),thickness,linetype);		

imshow("Middle line", bina);

imwrite("Midline.jpg",bina);	



			}//end if
					
}











//=========================================================================================
//==========================================================================================
//==========================================================================================








//==========================================================================================
//===============================Vertical Scan=======================================================
//==========================================================================================




void verticalScan(Mat& bina)
{
	int thickness=1;
	int linetype=8;


cout<<"-------------------------------Vertical Scan-----------------------------------"<<endl;

											int cur=0;;

											int startposition=0;

											int flag_first=1;

											Segment temp;

											vector<Segment>vSeg;
											vector<Point2d>vMid;
										
//cout<<raw.rows<<endl;
//cout<<raw.cols<<endl;

											vector<Triple> vTri;
											vector<Triple_Point>vTri_Point;
											Triple_Point tri_success={};
						

											int successnum=0;
int i;
int j;
							for ( j=0;j<bina.cols;j+=1)
							{
												
												int length=0;	
												
												int last=255;

												vSeg.clear(); // clear up all 

												//cout<<endl<<endl;
												//cout<<"============================================================================"<<endl;
												//cout<<"longitude scan in row="<<i<<endl;
			
												for( i=0;i<bina.rows;i++)
													{
														if(bina.at<uchar>(i,j)==0)
															
														{
																if(last==255)// ==1 true first comes here; ==0 not the first time

																{

																	length=0;// clear at beginning
																	startposition=i;

																	//cout<<"j="<<j<<endl;
								
																}	
																length++;
																last=0;

														}

														else

														{
															if(last==0 && (i+10)<bina.cols )
															{
																if(bina.at<uchar>(i+1,j)==255 && bina.at<uchar>(i+2,j)==255 && bina.at<uchar>(i+3,j)==255&& bina.at<uchar>(i+4,j)==255&& bina.at<uchar>(i+5,j)==255 ) // avoid poit white end so check next 5 points
																{
																	temp.startpos=startposition;
																	temp.length=length;

		

																	//length=0;  // big bug of clear here

																	vSeg.push_back(temp);
																}

															}
																
																last=255;


														}	

														

													  }	

	
										if(!vSeg.empty())
										{
//cout<<" vSeg not empty!"<<endl;
												Triple success={0,0,0};
												bool gotit=checkValidLine_vertical(vSeg, success,bina, i,j);

												if(gotit)
												{
													Point2d middle(j, ( vSeg.at(success.right).startpos + vSeg.at(success.left).startpos + vSeg.at(success.left).length)/2 ) ;
													vMid.push_back(middle);
										
													Point2d tri_start=Point2d(j,vSeg.at(success.left).startpos);
													Point2d tri_mid=Point2d(j,vSeg.at(success.middle).startpos);
													Point2d tri_last=Point2d(j,vSeg.at(success.right).startpos);

													tri_success={tri_start,tri_mid, tri_last};
												
													vTri_Point.push_back(tri_success);  ////

													cout<<"Left="<<vSeg.at(success.left).startpos<<"     right="<<vSeg.at(success.right).startpos<<endl;
													cout<<"Middle="<< middle<<endl;
													cout<<"row="<<i<<endl;
													cout<<"-------"<<endl;
													successnum++;
													

												
												}

										}




							}




				if(vMid.size()>0)
				{
													
cout<<"success num="<<successnum<<endl;
cout<<"all middle are---------"<<endl;		

for (int iter=0;iter<vMid.size();iter++)
	cout<<vMid.at(iter)<<endl;

cout<<endl;



//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//cluster


vector<Point2d>vMid_cluster;
double dist[vMid.size()-1];

double min_dist=100000;

for(int ii=0;ii<vMid.size()-1;ii++)
	{
		dist[ii]=vMid.at(ii+1).x- vMid.at(ii).x;
		cout<<"dist in vertical scan "<<ii<<"=  "<<dist[ii]<<endl; 
		if(dist[ii]<min_dist)
			min_dist=dist[ii];
	}



Cluster temp_cluster={0,0}; 
Cluster biggest_cluster={0,0};

double cluster_threshold=3; // at least 8 lines bundle

vector<Cluster>vClu;



int fflag_first=1;
for(int ii=0;ii<vMid.size()-1;ii++)
	{
		if(dist[ii]<(min_dist+5))
		{
			if(temp_cluster.length ==0)
			{
				temp_cluster.start=ii;
			}
			
			temp_cluster.length++;	

			if(ii==vMid.size()-1-1)
				if(temp_cluster.length > cluster_threshold)
						{
							vClu.push_back(temp_cluster);
						}	
		}

		else
			if(temp_cluster.length>0)
				{	
					if(temp_cluster.length > cluster_threshold)
						{
							vClu.push_back(temp_cluster);
						}

					temp_cluster.length=0;
				}
				
			
	}



for (int jj=0;jj<vClu.size();jj++)
	for(int ii=vClu.at(jj).start;ii< vClu.at(jj).start + vClu.at(jj).length;ii++)
	{
		
				vMid_cluster.push_back(vMid.at(ii));
line(bina,Point(vMid_cluster.back().x,0), Point( vMid_cluster.back().x,bina.rows), Scalar(0,255,0), thickness, linetype);
			
	}




//cluster end

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^






																
//line(bina,vMid_cluster.at(0), vMid_cluster.at(vMid_cluster.size()-1), Scalar(255,255,255),thickness,linetype);		

imshow("Middle line", bina);

imwrite("Midline.jpg",bina);	

				}//end if


					
}











//=========================================================================================
//==========================================================================================
//==========================================================================================







#include<stdio.h>
#include<sys/time.h>
#include<unistd.h>









int main()
{

											Mat raw=imread("recrec_two2.jpg",0);

											imshow("raw",raw);
										

											Mat bina(raw.rows, raw.cols, CV_8UC1, Scalar(0));



cout<<"----------------------------------binarized--------------------------------"<<endl;
										
											threshold( raw, bina, 160, 255,CV_THRESH_BINARY);
	
											
											imshow("binarized",bina);
											waitKey(0);

											imwrite("binarized.jpg",bina);

cout<<"----------------------------------binarized end--------------------------------"<<endl;



cout<<"--------------------------------- Add line---------------------------------"<<endl;

											Mat binaline=bina.clone();

											Point start(0,0), end(400,100);
											int thickness=1;
											int linetype=8;
											line(binaline,start, end, Scalar(0,0,0),thickness,linetype);

											imshow("imgline",binaline);


cout<<"------------------------------------------------------------------"<<endl;
//=====================measure time =========
struct timeval st, en;
gettimeofday( &st, NULL );
printf("start : %d s  and %d us \n", st.tv_sec, st.tv_usec);


//=====================end of measure time ======

					
											longitudeScan(bina);												
											//}											
											  
											verticalScan(bina);




//=====================measure time =========

gettimeofday( &en, NULL );
printf("end : %d.%d \n", en.tv_sec, en.tv_usec);


double delta_time=en.tv_sec-st.tv_sec+  (en.tv_usec - st.tv_usec)*0.000001;

cout<<"delta_time="<<delta_time<<endl;

//=====================end of measure time ======

											vector<Segment>vLineSeg;

											
		
											waitKey(0);

											return 1;


}













































