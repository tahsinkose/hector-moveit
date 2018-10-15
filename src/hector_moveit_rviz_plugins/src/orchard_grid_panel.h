#ifndef ORCHARD_GRID_PANEL_H
#define ORCHARD_GRID_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <vector>
# include <rviz/panel.h>
# include <geometry_msgs/Point.h>
# include <std_msgs/Float64.h>
# include <QLabel>
# include <QGraphicsRectItem>
# include <QGraphicsProxyWidget>
#endif

namespace hector_moveit_rviz_plugins
{

class OrchardGridPanel: public rviz::Panel{
	Q_OBJECT
	public:
		OrchardGridPanel( QWidget* parent = 0 );
	protected:
		ros::NodeHandle nh_;
		ros::Subscriber ack_sub,rate_sub;
	private:
		std::vector<std::vector<QGraphicsRectItem*> > grid;
		QLabel* exploration_percentage;
		float exploration_rate;
		void gridFillCallback(const geometry_msgs::Point::ConstPtr& msg); 
		void percentageCallback(const std_msgs::Float64::ConstPtr& msg);
};
}//end namespace 


#endif
