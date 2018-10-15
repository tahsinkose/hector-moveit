#include <QVBoxLayout>
#include <QGraphicsView>
#include <QGraphicsScene>
#include "orchard_grid_panel.h"


namespace hector_moveit_rviz_plugins
{
	OrchardGridPanel::OrchardGridPanel( QWidget* parent )
	  : rviz::Panel( parent ){
		int grid_size;
		nh_.getParam("/grid_size",grid_size);
		ack_sub = nh_.subscribe<geometry_msgs::Point>("/orchard_grid_filler",10,&OrchardGridPanel::gridFillCallback,this);
		rate_sub = nh_.subscribe<std_msgs::Float64>("/orchard_exploration_rate",1,&OrchardGridPanel::percentageCallback,this);
		grid.resize(grid_size,std::vector<QGraphicsRectItem*>(grid_size));

		QGraphicsScene* scene = new QGraphicsScene(this);
		int height = 25;
		int width = 25;
		QPen blackPen(Qt::black);
		for(int i=0;i<grid_size;i++){
			for(int j=0;j<grid_size;j++){
				
				QGraphicsRectItem* rect = scene->addRect(i*width,j*height,width,height,blackPen);
				//rect->setBrush(Qt::blue);
				//rect->update();
				grid[i][j] = rect;
			}
		}
		this->exploration_percentage = new QLabel;
		this->exploration_percentage->setStyleSheet("font-weight: bold; color: green");
		this->exploration_percentage->setAlignment(Qt::AlignCenter);
		QFont f("Times",16,QFont::Bold);
		this->exploration_percentage->setFont(f);
		this->exploration_percentage->setText("Exploration: ");
		QGraphicsProxyWidget* info = scene->addWidget(this->exploration_percentage);
		info->setPos(300,400);
		QGraphicsView* view = new QGraphicsView(scene,this);
		QHBoxLayout* frame = new QHBoxLayout(this);
		frame->addWidget(view);
		QVBoxLayout* layout = new QVBoxLayout(this);
		
		
		
		layout->addLayout(frame);
		setLayout(layout);
	}
	void OrchardGridPanel::gridFillCallback(const geometry_msgs::Point::ConstPtr& msg){
		grid[msg->y][msg->x]->setBrush(Qt::blue);
		grid[msg->y][msg->x]->update();
	}
	void OrchardGridPanel::percentageCallback(const std_msgs::Float64::ConstPtr& msg){
		QString qss = QString("Exploration: %1").arg(msg->data);
		this->exploration_percentage->setText(qss);
		QFont f("Times",10,QFont::Bold);
		this->exploration_percentage->setFont(f);
		this->exploration_percentage->update();
	}
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_moveit_rviz_plugins::OrchardGridPanel,rviz::Panel )
