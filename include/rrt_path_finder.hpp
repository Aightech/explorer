#ifndef RRT_PATH_FINDER_HPP
#define RRT_PATH_FINDER_HPP

/*
 * Standard libs include
 */
#include <vector>

/*
 * OpenCV includes
 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define MAX_INC 30
#define PRECISION 0.001

#define LAGRANGE 0
#define BEZIER 1


typedef struct _Vertex
{
  double data[2];
  struct _Vertex * parent;
  double dist;
  unsigned int index;
}Vertex;

typedef struct {
        Vertex P[4];
  } Bezier;

cv::Point2f vertex_to_point2f(const Vertex& v);
std::string type2str(int type) ;
int is_goal_reachable(const Vertex& target, const std::vector<Vertex*>& lv, const cv::Mat& im);
double dist2(const Vertex& v1, const Vertex& v2);
double no_wall_between(const Vertex& v1, const Vertex& v2, const cv::Mat& im);
Vertex* nearest_vertex(const Vertex& qrand, const std::vector<Vertex*>& lv);
void rand_free_conf(Vertex& qrand, int height, int width);
Vertex* new_conf(const Vertex& qrand, Vertex& qnear, double *dq, const cv::Mat& im);

void find_path(cv::Mat image, Vertex* parent, Vertex* qnew, std::vector<Vertex*>& path, bool with_gui=false);
void straighten_path(cv::Mat image, const cv::Mat emptyMap, std::vector<Vertex*>& path, bool with_gui=false);
void smoothen_path(cv::Mat image, const cv::Mat emptyMap, std::vector<Vertex*>& path,int type_interpolation=BEZIER, bool with_gui=false);

void linear_interpol_path(cv::Mat image, const cv::Mat emptyMap, std::vector<Vertex*>& path, bool with_gui=false);

#endif
