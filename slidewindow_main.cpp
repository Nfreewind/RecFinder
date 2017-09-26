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

	Line(){}

	Line(Point2d startp_, Point2d endp_)
	{
		startp=startp_;
		endp=endp_;

		k=(endp.y -startp.y)/(endp.x-startp.x);

		b=startp.y - k*startp.x;
	}



	void LinePrint()
	{
		cout<<"line.startp="<<startp<<"  line.endp="<<endp<<endl;
	}	



	Point2d startp;
	Point2d endp;
	double k, b;  // lean rate (slope), cut distance(intercept).
	
};


double min4(double d1, double d2, double d3, double d4)
{
	double temp=d1;
	if(temp>d2) temp=d2;
	if(temp>d3) temp=d3;
	if(temp>d4) temp=d4;
	return temp;
}

double max4(double d1, double d2, double d3, double d4)
{
	double temp=d1;
	if(temp<d2) temp=d2;
	if(temp<d3) temp=d3;
	if(temp<d4) temp=d4;
	return temp;
}



//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
/////////////////////////////calculate two lines' intersection point 
 
bool intersectionOf2lines(Line& line1, Line& line2, Point2d& output)  // bool is to check inrange or out of range

{
	output.x= (line1.b- line2.b)/(line2.k - line1.k);
	
	output.y= (line1.b*line2.k - line2.b*line1.k)/(line2.k-line1.k);

	
/////////check range 


	if(output.x <= max(line1.startp.x, line1.endp.x)  && output.x >= min(line1.startp.x, line1.endp.x)  &&
	   output.x <= max(line2.startp.x, line2.endp.x)  && output.x >= min(line2.startp.x, line2.endp.x)  &&
	   output.y <= max(line1.startp.y, line1.endp.y)  && output.y >= min(line1.startp.y, line1.endp.y)  &&
    	   output.y <= max(line2.startp.y, line2.endp.y)  && output.y >= min(line2.startp.y, line2.endp.y)  )     // embedded in the litte range
		 
		return 1;
	else
		return 0;
		
}
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------





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
		double ratioCheckLimit= (double)(meanLength/5.0);  //unit:pixels

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




void longitudeScan(Mat& bina, vector<Point2d>& vMid_cluster, Line& longSc_line1, Line& longSc_line2 )
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


//vector<Point2d>vMid_cluster;
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
//line(bina,Point(0,vMid_cluster.back().y), Point( bina.cols,vMid_cluster.back().y), Scalar(0,255,0), thickness, linetype);
			
	}


//cluster end

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^





////////line assign value///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Point2d line1_startp=vMid.at(vClu.at(0).start);
Point2d line1_endp=vMid.at(vClu.at(0).start + vClu.at(0).length -1);

longSc_line1=Line(line1_startp,line1_endp);




Point2d line2_startp=vMid.at(vClu.at(1).start);
Point2d line2_endp=vMid.at(vClu.at(1).start + vClu.at(1).length -1);

longSc_line2=Line(line2_startp,line2_endp);





////////line assign value end ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%







//////////






//draw middle line

for (int jj=0;jj<vClu.size();jj++)
{
	line(bina,   vMid.at(vClu.at(jj).start),  vMid.at(vClu.at(jj).start+vClu.at(jj).length -1), Scalar(255,255,255), thickness, linetype);	
}

																
//line(bina,vMid_cluster.at(0), vMid_cluster.at(vMid_cluster.size()-1), Scalar(255,255,255),thickness,linetype);		

imshow("Middle line long", bina);

imwrite("Midline.jpg",bina);	







			}//end if
					
}











//=========================================================================================
//==========================================================================================
//==========================================================================================








//==========================================================================================
//===============================Vertical Scan=======================================================
//==========================================================================================




void verticalScan(Mat& bina, vector<Point2d>& vMid_cluster, Line& verSc_line1, Line& verSc_line2)
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


//vector<Point2d>vMid_cluster;
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

double cluster_threshold=2; // at least 8 lines bundle

vector<Cluster>vClu;



int fflag_first=1;
for(int ii=0;ii<vMid.size()-1;ii++)
	{
		if(dist[ii]<(min_dist+6))
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
//line(bina,Point(vMid_cluster.back().x,0), Point( vMid_cluster.back().x,bina.rows), Scalar(0,255,0), thickness, linetype);
//line(bina,vMid_cluster.back(),  vMid_cluster.at(end()), Scalar(0,255,0), thickness, linetype);			
	}






//cluster end

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


////////line assign value///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cout<<"vertical vClu.size()="<<vClu.size()<<endl;
cout<<"vClu.at(0).length="<< vClu.at(0).length<<endl;
cout<<"vClu.at(1).length="<< vClu.at(1).length<<endl;
//cout<<"vClu.at(2).length="<< vClu.at(2).length<<endl;

cout<<"vClu.at(0).start="<<vClu.at(0).start<<endl;
cout<<"vClu.at(1).start="<<vClu.at(1).start<<endl;
//cout<<"vClu.at(2).start="<<vClu.at(2).start<<endl;

Point2d line1_startp=vMid.at(vClu.at(0).start);
Point2d line1_endp=vMid.at(vClu.at(0).start + vClu.at(0).length -1);

verSc_line1=Line(line1_startp,line1_endp);




Point2d line2_startp=vMid.at(vClu.at(1).start);
Point2d line2_endp=vMid.at(vClu.at(1).start + vClu.at(1).length -1);

verSc_line2=Line(line2_startp,line2_endp);





////////line assign value end ///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^fuck"<<endl;
cout<<"vClu.size()="<<vClu.size()<<endl;

//draw middle line

for (int jj=0;jj<vClu.size();jj++)
{
	cout<<jj<<"  vMid.at(vClu.at(jj).start)="<<vMid.at(vClu.at(jj).start)<<endl;
	line(bina,   vMid.at(vClu.at(jj).start),  vMid.at(vClu.at(jj).start+vClu.at(jj).length -1), Scalar(255,255,255), thickness, linetype);	
}


																
//line(bina,vMid_cluster.at(0), vMid_cluster.at(vMid_cluster.size()-1), Scalar(255,255,255),thickness,linetype);		

imshow("Middle line ver", bina);

imwrite("Midline.jpg",bina);	

				}//end if


					
}











//=========================================================================================
//==========================================================================================
//==========================================================================================




//=========================================================================================
//==========================================================================================
//==========================================================================================



///mathvec





//=========================================================================================
//==========================================================================================
//==========================================================================================
//typedef Point2d Vec;




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
										
											threshold( raw, bina, 165, 255,CV_THRESH_BINARY);   // for recrec_two2.jpg best 165
	
											
											imshow("binarized",bina);
											waitKey(0);

											imwrite("binarized.jpg",bina);

cout<<"----------------------------------binarized end--------------------------------"<<endl;



cout<<"--------------------------------- Add line---------------------------------"<<endl;

											Mat binaline=bina.clone();

											Point start(0,0), end(400,100);
											int thickness=1;
											int linetype=8;
											//line(binaline,start, end, Scalar(0,0,0),thickness,linetype);

											imshow("imgline",binaline);


cout<<"------------------------------------------------------------------"<<endl;
//=====================measure time =========
struct timeval st, en;
gettimeofday( &st, NULL );
printf("start : %d s  and %d us \n", st.tv_sec, st.tv_usec);


//=====================end of measure time ======

											vector<Point2d>Vmid_correct_long;
											vector<Point2d>Vmid_correct_vert;

											Line longSc_line1;
											Line longSc_line2;
											Line verSc_line1;
											Line verSc_line2;								
		
											Mat bina_4long=bina.clone();					
											
											longitudeScan(bina_4long,Vmid_correct_long, longSc_line1, longSc_line2);  ///////////////////	           			
											//}											
																						

											Mat bina_4ver=bina.clone();
											verticalScan(bina_4ver, Vmid_correct_vert, verSc_line1, verSc_line2);  /////////////////

											longSc_line1.LinePrint();
											longSc_line2.LinePrint();

				
											verSc_line1.LinePrint();
											verSc_line2.LinePrint();

cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least, we have gotten four line segments.^^^^^^^^^^^^^^^^^^^^"<<endl;
cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least, we have gotten four line segments.^^^^^^^^^^^^^^^^^^^^"<<endl;
cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least, we have gotten four line segments.^^^^^^^^^^^^^^^^^^^^"<<endl;
cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least, we have gotten four line segments.^^^^^^^^^^^^^^^^^^^^"<<endl;

		
					
											bool b_l1_v1, b_l1_v2, b_l2_v1, b_l2_v2;

											Point2d p_l1_v1, p_l1_v2, p_l2_v1, p_l2_v2;
	
											b_l1_v1=intersectionOf2lines(longSc_line1, verSc_line1,  p_l1_v1); 
											b_l1_v2=intersectionOf2lines(longSc_line1, verSc_line2,  p_l1_v2); 
											b_l2_v1=intersectionOf2lines(longSc_line2, verSc_line1,  p_l2_v1); 
											b_l2_v2=intersectionOf2lines(longSc_line2, verSc_line2,  p_l2_v2); 

	

											cout<<"b_l1_v1="<<b_l1_v1<<endl; cout<<"p_l1_v1="<<p_l1_v1<<endl;
											cout<<"b_l1_v2="<<b_l1_v2<<endl; cout<<"p_l1_v2="<<p_l1_v2<<endl;
											cout<<"b_l2_v1="<<b_l2_v1<<endl; cout<<"p_l2_v1="<<p_l2_v1<<endl;
											cout<<"b_l2_v2="<<b_l2_v2<<endl; cout<<"p_l2_v2="<<p_l2_v2<<endl;



											vector<Point2d>centers;// center1, center2;
											if(b_l1_v1==1)	centers.push_back(p_l1_v1);
											if(b_l1_v2==1)	centers.push_back(p_l1_v2);
											if(b_l2_v1==1)	centers.push_back(p_l2_v1);
											if(b_l2_v2==1)	centers.push_back(p_l2_v2);

						
											Point2d center1, center2;
							


											center1=centers.at(0);
											center2=centers.at(1);

cout<<"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$   center we got it! $$$$$$$$$$"	<<endl;

cout<<"center1="<<center1<<endl;
cout<<"center2="<<center2<<endl;


cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least, we have gotten two centers.^^^^^^^^^^^^^^^^^^^^"<<endl;
cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least, we have gotten two centers.^^^^^^^^^^^^^^^^^^^^"<<endl;
cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least, we have gotten two centers.^^^^^^^^^^^^^^^^^^^^"<<endl;
cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least, we have gotten two centers.^^^^^^^^^^^^^^^^^^^^"<<endl;





	Mat bina_show=bina.clone();

	line(bina_show,center1, center2, Scalar(255,255,255),thickness,linetype);

	imshow("centercenter",bina_show);




cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;


cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  begin to judge which is left center  -------"<<endl;

imshow("bina_raw",bina);

													Mat bina_check=bina.clone();

													Point2d vec12=center2-center1;                     // math vec from center 1 to  center 2
													Point2d vec21=-vec12;
													double dist12=sqrt(vec12.x*vec12.x +vec12.y*vec12.y );

													Point2d normal_vec12=(1.0/dist12) * vec12;
													Point2d normal_vec21=(1.0/dist12) * vec21;


													Point2d step_vec12=(1.0/21.0) * vec12;   // 1.0 and 1 very different in int and double, especially in division  // one step is one little black square	
													Point2d step_vec21=-step_vec12;

													// check whether center 1 or center 2 is left center
													int left_index=0; // 1,2


													Point2d check1=center1+5*step_vec12;
													Point2d check2=center2+5*step_vec21;


//cout<<"check1="<<check1<<endl;
//cout<<"check2="<<check2<<endl;

Point2d step_vec_left;
Point2d center_left;
													if(bina_check.at<uchar>(cvRound(check1.y), cvRound(check1.x)) ==255 && bina_check.at<uchar>(cvRound(check2.y), cvRound(check2.x)) ==0 )
													{
														cout<<"check matches and center2=   "<<center2<<"  is left center"<<endl;
														left_index=2;
														
														step_vec_left=step_vec21;
														center_left=center2;

														
													}
													else
														if(bina_check.at<uchar>(cvRound(check1.y), cvRound(check1.x)) ==0 && bina_check.at<uchar>(cvRound(check2.y), cvRound(check2.x)) ==255 )
															{
																cout<<"check matches and center1=   "<<center1<<"  is left center"<<endl;
																left_index=1;

																step_vec_left=step_vec12;
																center_left=center1;
															}

														else
															cout<<"check unmatched !"<<endl;




													cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least,we know who is the left center.^^^^^^^^^^^^^^^^^^^^"<<endl;
													cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least,we know who the left center.^^^^^^^^^^^^^^^^^^^^"<<endl;
													cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least,we know who the left center.^^^^^^^^^^^^^^^^^^^^"<<endl;
													cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least,we know who the left center.^^^^^^^^^^^^^^^^^^^^"<<endl;








cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;


cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  begin to get orthogonal step vec -------"<<endl;






Point2d vec_orthogonal(1, - (step_vec_left.x/step_vec_left.y));

double norm_vec_orth= sqrt(1+  (step_vec_left.x/step_vec_left.y)*(step_vec_left.x/step_vec_left.y));

double norm_step=sqrt(step_vec_left.x*step_vec_left.x + step_vec_left.y*step_vec_left.y);

Point2d normal_vec_orthogonal= (1.0/norm_vec_orth) * vec_orthogonal;

Point2d step_vec_orthogonal_temp= norm_step * normal_vec_orthogonal;// righthand rotation order with 90 deg clockwise

Point2d step_vec_orthogonal;
//judge up or down 


Point2d checkedgepoint1=center_left + 5*step_vec_left; //  choose 3 points with gap 3*step
Point2d checkedgepoint2=center_left + 5*step_vec_left + 3*step_vec_orthogonal_temp;
Point2d checkedgepoint3=center_left + 5*step_vec_left + 6*step_vec_orthogonal_temp;

if(  bina_check.at<uchar>( cvRound(checkedgepoint1.y), cvRound(checkedgepoint1.x) ) == 0 && bina_check.at<uchar>( cvRound(checkedgepoint2.y), cvRound(checkedgepoint2.x) ) == 0 && bina_check.at<uchar>( cvRound(checkedgepoint2.y), cvRound(checkedgepoint2.x) ) == 0   )
	step_vec_orthogonal=step_vec_orthogonal_temp;

else
	step_vec_orthogonal=-step_vec_orthogonal_temp;


cout<<"step_vec_orthogonal="<<step_vec_orthogonal<<endl;


													cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least,we know orthogonal stepsize vector and in right direction(Barcode Reference y axis).^^^^^^^^^^^^^^^^^^^^"<<endl;
													cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least,we know orthogonal stepsize vector and in right direction(Barcode Reference y axis).^^^^^^^^^^^^^^^^^^^^"<<endl;
													cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least,we know orthogonal stepsize vector and in right direction(Barcode Reference y axis).^^^^^^^^^^^^^^^^^^^^"<<endl;
													cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^up to now, at least,we know orthogonal stepsize vector and in right direction(Barcode Reference y axis).^^^^^^^^^^^^^^^^^^^^"<<endl;
													// calculate the step vec orthogonal







cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<endl;


cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  begin to calculate all the 0s and 1s -------"<<endl;


Mat bina_final=bina.clone();

Mat decodeResult= (Mat_<uchar>(10,10)<<0);
cout<<"decodeResult="<<decodeResult<<endl;


for(int fi=0;fi<10;fi++)
	for(int fj=0;fj<10;fj++)
	
	{
		Point2d temp= center_left + 6*step_vec_left + fj*step_vec_left + fi * step_vec_orthogonal;
	
		if(bina_final.at<uchar>( cvRound(temp.y) , cvRound(temp.x) ) ==255 )
			decodeResult.at<uchar>(fi,fj)=1;
		else
			decodeResult.at<uchar>(fi,fj)=0;	
		
		
	}





cout<<"==========================fuck out the final fuck up 0s and 1s ============================================"<<endl;

cout<<"decodeResult="<<endl<<decodeResult<<endl;




//=====================measure time =========

gettimeofday( &en, NULL );
printf("end : %d.%d \n", en.tv_sec, en.tv_usec);


double delta_time=en.tv_sec-st.tv_sec+  (en.tv_usec - st.tv_usec)*0.000001;

cout<<"delta_time="<<delta_time<<endl;

//=====================end of measure time ======



////////////////////////////////////////////////////////////////////////////////////////////draw out  mid points

///////////////////////////////////drawmidline_longSc
/*		
											Mat drawmidline_long(bina.rows, bina.cols,CV_8UC1, Scalar(0));
											

											for(int i_=0;i_<Vmid_correct_long.size();i_++)
												{
													int temp_x=cvRound(Vmid_correct_long.at(i_).x);
													int temp_y=cvRound(Vmid_correct_long.at(i_).y);

													drawmidline_long.at<uchar>(temp_y,temp_x)=255;
												}

											imshow("drawmidline_long",drawmidline_long);

									
////////////////////////////////////drawmidline_verSc


									Mat drawmidline_ver(bina.rows, bina.cols,CV_8UC1, Scalar(0));
											

											for(int i_=0;i_<Vmid_correct_vert.size();i_++)
												{
													int temp_x=cvRound(Vmid_correct_vert.at(i_).x);
													int temp_y=cvRound(Vmid_correct_vert.at(i_).y);

													drawmidline_ver.at<uchar>(temp_y,temp_x)=255;
												}

											imshow("drawmidline_ver",drawmidline_ver);

*/







///////////////////////////////Intersection








/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*


											line(bina,Point(249,215), Point(448,272), Scalar(0,0,0),thickness,linetype);
					
											imshow("bina middle connect",bina); 
											imwrite("middleconnect.jpg",bina);
			
										

//=================================decode the 0s and 1s


								Point2d V=Point2d(249-448,215-272);
			
								Point2d Vstep;
								Vstep.x=(1.0/21.0)* V.x;
								Vstep.y=(1.0/21.0)* V.y;

								Point A=Point2d(448,272);
				
								bool firstRow[10];

								Point2d enn(-0.27536,0.96134);
								Point2d stepDown;
								stepDown.x=9.857*enn.x;
								stepDown.y= 9.857*enn.y;
	

						for (int jjj=0;jjj<10;jjj++)
								
						{			
							for(int iii=0;iii<10;iii++)
			
									{
										Point2d tempd;
										tempd.x= A.x+ (iii+6) *Vstep.x +jjj*stepDown.x;
										tempd.y= A.y+ (iii+6) *Vstep.y +jjj*stepDown.y;
										Point tempi;
										tempi.x= cvRound(tempd.x);
										tempi.y= cvRound(tempd.y);
										
										//cout<<"tempi="<<tempi<<endl;

										if(binaline.at<uchar>(tempi.y,tempi.x) == 255)
											firstRow[iii]=1;
										else
											firstRow[iii]=0;

										cout<<firstRow[iii]<<"  ";
											
									}


							cout<<"-------------"<<endl;
						}
											

*/		
											waitKey(0);

											return 1;


}














































