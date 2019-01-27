#include "rrt_path_finder.hpp"
#include <iostream>

cv::Point2f vertex_to_point2f(const Vertex& v)
{
  return cv::Point2f(v.data[0], v.data[1]);
}

std::string type2str(int type) 
{
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) 
  {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


int is_goal_reachable(const Vertex& target, const std::vector<Vertex*>& lv, const cv::Mat& im)
{
  double min_dist = DBL_MAX;
  double dist;
  int ind = -1;
  for(unsigned int i = 0 ; i < lv.size() ; i++)
  {
    dist = sqrt(dist2(target,*lv[i]));
    if(no_wall_between(target,*lv[i], im)>=dist && min_dist > dist)
    {
      min_dist = dist;
      ind = i;
    }
  }
  return ind;
}

double dist2(const Vertex& v1, const Vertex& v2)
{
    return ((v1.data[0]-v2.data[0])*(v1.data[0]-v2.data[0]) + (v1.data[1]-v2.data[1])*(v1.data[1]-v2.data[1])); 
}

double no_wall_between(const Vertex& v1, const Vertex& v2, const cv::Mat& im)
{
  uint8_t* pixelPtr = (uint8_t*)im.data;
  int cn = im.channels();

  double dist = dist2(v1, v2);

  double vx = -(v1.data[0]-v2.data[0])/sqrt(dist); //vecteur unitaire selon x
  double vy = -(v1.data[1]-v2.data[1])/sqrt(dist); //vecteur unitaire selon y

  double i = PRECISION;

  while((vx*i*vx*i) + (vy*i*vy*i) < dist)
  {
    if(pixelPtr[(int)(ceil((v1.data[1] + i*vy))*im.cols*cn + ceil((v1.data[0] + i*vx))*cn)] == 0)
      return i - PRECISION;
    i += PRECISION;
  }
  return i;
}

Vertex* nearest_vertex(const Vertex& qrand, const std::vector<Vertex*>& lv)
{
  int min_index = 0;
  double min = DBL_MAX;
  double dist;

  for(unsigned int i = 0 ; i < lv.size() ; i++)
    if(min > (dist = dist2(qrand, *lv[i])))
    {
        min = dist;
        min_index = i;
    }
  return lv[min_index];
}

void rand_free_conf(Vertex& qrand, int height, int width)
{
  qrand.data[1] = rand()%height;
  qrand.data[0] = rand()%width;
  qrand.parent = NULL;
}

Vertex* new_conf(const Vertex& qrand, Vertex& qnear, double *dq, const cv::Mat& im)
{
  double dist = dist2(qrand, qnear);
  double vx = qnear.data[0] + (qrand.data[0]-qnear.data[0])/sqrt(dist)*(*dq);
  double vy = qnear.data[1] + (qrand.data[1]-qnear.data[1])/sqrt(dist)*(*dq);
  vx = (vx < 0) ? 0 : vx;
  vy = (vy < 0) ? 0 : vy;
  vx = (vx > im.cols) ? im.cols : vx;
  vy = (vy > im.rows) ? im.rows : vy;
  Vertex* v = new Vertex{{vx, vy},&qnear, qnear.dist + *dq, qnear.index+1};
  if((*dq = no_wall_between(*v, qnear, im)) == MAX_INC)
    return v;
  else if(*dq > 0)
  {
    v->data[0] = qnear.data[0] + (qrand.data[0]-qnear.data[0])/sqrt(dist)*(*dq);
    v->data[1] = qnear.data[1] + (qrand.data[1]-qnear.data[1])/sqrt(dist)*(*dq);
    v->dist = qnear.dist + *dq;
    return v;
  }
  return NULL;
}





void find_path(cv::Mat image, Vertex* parent, Vertex* qnew, std::vector<Vertex*>& path, bool with_gui)
{
  while(parent!=NULL)
  {
    path[qnew->index]=qnew;
    cv::line(image, vertex_to_point2f(*parent), vertex_to_point2f(*qnew), cv::Scalar(0,255,0), 2, CV_AA);
    qnew=parent;
    parent=qnew->parent;
    if(with_gui)
	{
    	cv::imshow( "Display window2", image );
    	cv::waitKey(1);
    }
  }
  path[qnew->index]=qnew;
}

void straighten_path(cv::Mat image, const cv::Mat emptyMap, std::vector<Vertex*>& path, bool with_gui)
{
 	unsigned int i=0;
  while(i < path.size()-2)
  {
		for(unsigned int j=0; j < path.size()-i-3; j++)
		{
			int size = path.size();
			double dist = sqrt(dist2(*path[j],*path[size-i-1]));
			if(no_wall_between(*path[j],*path[size-i-1], emptyMap)>=dist)
			{
				cv::line(image, vertex_to_point2f(*path[j]), vertex_to_point2f(*path[size-i-1]), cv::Scalar(0,200,0), 2, CV_AA);
				path.erase(path.begin()+j+1, path.begin() + size-i-1);
				break;
			}
			
		}
		i++;
		if(with_gui)
		{
			cv::imshow( "Display window2", image );
			cv::waitKey(1);
		}
	}
}

void linear_interpol_path(cv::Mat image, const cv::Mat emptyMap, std::vector<Vertex*>& path, bool with_gui)
{
    std::cout << "Starting interpol path" << std::endl;
	int N = path.size();
	double nbPt = 5;
	double dt[path.size()-1];

	std::vector<Vertex*> new_path;

	for(int k=0; k < path.size()-1; k++)
			dt[k] = sqrt(dist2(*path[k],*path[k+1]))/200.;
    std::cout << "Step 1 ok" << std::endl;

	std::vector<Vertex*>::iterator it=path.begin();
	for(int i =0;i<N-1 ; i++)
		{
			double vx = path[i]->data[0];
			double vy = path[i]->data[1];
			double dx = path[i+1]->data[0];
			double dy = path[i+1]->data[1];
			
			for(double t = 0; t <=  1; t += 1/(nbPt+dt[i]))
				{
				    std::cout << "avant" << std::endl;

					new_path.push_back(new Vertex{{t*dx + (1-t)*vx,t*dy +(1-t)*vy},NULL, 1, 0});
				    std::cout << "apres" << std::endl;
				}
		}
    std::cout << "Step 2 ok" << std::endl;

	for(int k=0; k < path.size(); k++)
		{
			cv::circle(image, cv::Point2f(path[k]->data[0], path[k]->data[1]), 3, cv::Scalar(0,200,0), -1, 10, 0);
			if(with_gui)
			{
				cv::imshow( "Display window", image );
				cv::waitKey(1);
			}
		}
	path.erase(path.begin(),path.end());	
	for(int i = 0; i<new_path.size();i++)
		path.push_back(new_path[i]);

    std::cout << "Step 3 ok" << std::endl;


}

void smoothen_path(cv::Mat image, const cv::Mat emptyMap, std::vector<Vertex*>& path,int type_interpolation, bool with_gui)
{
    std::cout << "Starting smoothen path" << std::endl;

	if(type_interpolation == LAGRANGE)
		{
			double Ldenom[path.size()];
			//compute the differents time spans between each points 
			double dt[path.size()];
			dt[0]=0;
			for(int k=1; k < path.size(); k++)
				dt[k] = k;
	
			Ldenom[0]=1;
			for(int k=0; k < path.size();Ldenom[++k]=1)
				for(int j=0; j < path.size(); j++)
					if(j!=k)
						Ldenom[k] = Ldenom[k] *(dt[k] - dt[j]);
				
			double incT = 0.1;
			for(double t = 0; t <=  dt[path.size()-1]; t += incT)
				{
					double fx = 0,fy=0;
					for(int k=0; k < path.size(); k++)
						{
							double pdt = 1;
							for(int j=0; j < path.size(); j++)
								{
									if(j!=k)
										pdt *= (t - dt[j]); 
								}
							//std::cout << "pdt k=" << k << " ( " << pdt << " )" << std::endl;
							fx += path[k]->data[0]*pdt/Ldenom[k];
							fy += path[k]->data[1]*pdt/Ldenom[k];
							//std::cout << "k=" << k << " fx " << fx << " fy" << fy << std::endl;
						}
					cv::circle(image, cv::Point2f(fx, fy), 3, cv::Scalar(0,0,0), -1, 8, 0);
					if(with_gui)
					{
						cv::imshow( "Display window", image );
						cv::waitKey(1);
					}
					std::cout << "t=" << t << " ( " << fx << " , " << fy << " )" << std::endl; 
				}

		}
	else if(type_interpolation == BEZIER)
		{
			int N = path.size();
			double Tx[N][N], Ty[N][N]; 
			double nbPt = 100;
			for(int j=0; j<N; j++)
				{
					Tx[0][j]=path[j]->data[0];
					Ty[0][j]=path[j]->data[1];
					//std::cout << "init=" << j << " ( " << Tx[0][j] << " , " << Ty[0][j] << " )" << std::endl;
				}
			path.erase(path.begin(),path.end());
			for(double t = 0; t <=  1; t += 1/nbPt)
				{	
					for(int i=1; i<N; i++)
						{
							for(int j=i; j<N; j++)
								{
									Tx[i][j] = t*Tx[i-1][j-1] + (1-t)*Tx[i-1][j];//T^i_j
									Ty[i][j] = t*Ty[i-1][j-1] + (1-t)*Ty[i-1][j];//T^i_j
								}
						}
					path.push_back(new Vertex{{Tx[N-1][N-1], Ty[N-1][N-1]},NULL, 1, 0});
					cv::circle(image, cv::Point2f(Tx[N-1][N-1], Ty[N-1][N-1]), 3, cv::Scalar(0,0,0), -1, 8, 0);
					if(with_gui)
					{
						cv::imshow( "Display window", image );
						cv::waitKey(1);
					}
					//std::cout << "t=" << t << " ( " << Tx[N-1][N-1] << " , " << Ty[N-1][N-1] << " )" << std::endl;
				}
		}
}



