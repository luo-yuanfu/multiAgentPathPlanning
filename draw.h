/*
 * draw.h
 *
 *  Created on: Nov 12, 2014
 *      Author: Luo Yuanfu
 */

#ifndef DRAW_H_
#define DRAW_H_

#include <graphics.h>
#include <X11/Xlib.h>

class vect//vector
{public:
    double x;
    double y;
    void setValue(double x,double y)
    {
    	this->x=x;
    	this->y=y;
    }
};

class obstacles
{public:
    double x0;
    double y0;
    double x1;
    double y1;

    void setValue(double x0,double y0,double x1,double y1)
    {
    	this->x0=x0;
    	this->y0=y0;
    	this->x1=x1;
    	this->y1=y1;
    }
};

class robot_pos
{public:
	double x1;
	double y1;
	double x2;
	double y2;
	double x3;
	double y3;
	double x4;
	double y4;
	double x5;
	double y5;

	void setValue(double x1,double y1,double x2,double y2,double x3, double y3,double x4,double y4,double x5, double y5)
	{
	    this->x1=x1;
	    this->y1=y1;
	    this->x2=x2;
	    this->y2=y2;
	    this->x3=x3;
	   	this->y3=y3;
	   	this->x4=x4;
	   	this->y4=y4;
	   	this->x5=x5;
	   	this->y5=y5;
	}
};

std::vector<obstacles> obsts;
std::vector<robot_pos> path;

void drawPath(std::vector<robot_pos>path,double env_width,double env_height,int agent_num)
{
	double graph_width=getmaxx();
	double graph_height=getmaxy();

	setcolor(YELLOW);
	circle(int(path.at(0).x1/env_width*graph_width),int(path.at(0).y1/env_height*graph_height),3);

	for(int i=1;i<path.size();i++)
	{

		line(int(path.at(i).x1/env_width*graph_width),int(path.at(i).y1/env_height*graph_height),int(path.at(i-1).x1/env_width*graph_width),int(path.at(i-1).y1/env_height*graph_height));
		circle(int(path.at(i).x1/env_width*graph_width),int(path.at(i).y1/env_height*graph_height),3);
	}

	setcolor(GREEN);
	circle(int(path.at(0).x2/env_width*graph_width),int(path.at(0).y2/env_height*graph_height),3);

	if(agent_num==1) return;

	for(int i=1;i<path.size();i++)
	{

		line(int(path.at(i).x2/env_width*graph_width),int(path.at(i).y2/env_height*graph_height),int(path.at(i-1).x2/env_width*graph_width),int(path.at(i-1).y2/env_height*graph_height));
		circle(int(path.at(i).x2/env_width*graph_width),int(path.at(i).y2/env_height*graph_height),3);
	}


	if(agent_num==2) return;

	setcolor(RED);
	circle(int(path.at(0).x3/env_width*graph_width),int(path.at(0).y3/env_height*graph_height),3);

	for(int i=1;i<path.size();i++)
	{

		line(int(path.at(i).x3/env_width*graph_width),int(path.at(i).y3/env_height*graph_height),int(path.at(i-1).x3/env_width*graph_width),int(path.at(i-1).y3/env_height*graph_height));
		circle(int(path.at(i).x3/env_width*graph_width),int(path.at(i).y3/env_height*graph_height),3);
	}

	if(agent_num==3) return;

	setcolor(BLUE);
	circle(int(path.at(0).x4/env_width*graph_width),int(path.at(0).y4/env_height*graph_height),3);

	for(int i=1;i<path.size();i++)
	{

		line(int(path.at(i).x4/env_width*graph_width),int(path.at(i).y4/env_height*graph_height),int(path.at(i-1).x4/env_width*graph_width),int(path.at(i-1).y4/env_height*graph_height));
		circle(int(path.at(i).x4/env_width*graph_width),int(path.at(i).y4/env_height*graph_height),3);
	}

	if(agent_num==4) return;

	setcolor(BROWN);
	circle(int(path.at(0).x5/env_width*graph_width),int(path.at(0).y5/env_height*graph_height),3);

	for(int i=1;i<path.size();i++)
	{

			line(int(path.at(i).x5/env_width*graph_width),int(path.at(i).y5/env_height*graph_height),int(path.at(i-1).x5/env_width*graph_width),int(path.at(i-1).y5/env_height*graph_height));
			circle(int(path.at(i).x5/env_width*graph_width),int(path.at(i).y5/env_height*graph_height),3);
	}



}




void drawObsts(double env_width,double env_height)
{
	double graph_width=getmaxx();
	double graph_height=getmaxy();

	setcolor(WHITE);
//	setfillstyle(BROWN,5,NULL);
	for(int i=0;i<obsts.size();i++)
	{
		//fillpoly(2,int(obsts.at(i).x0*500),int(obsts.at(i).y0*500),int(obsts.at(i).x1*500),int(obsts.at(i).y1*500));
		rectangle(int(obsts.at(i).x0/env_width*graph_width),int(obsts.at(i).y0/env_height*graph_height),int(obsts.at(i).x1/env_width*graph_width),int(obsts.at(i).y1/env_height*graph_height));
		//bar(int(obsts.at(i).x0*graph_width),int(obsts.at(i).y0*graph_height),int(obsts.at(i).x1*graph_width),int(obsts.at(i).y1*graph_height));
	}
}

void loadData(int agent_num)
  {
  	FILE *p;
  	char temp[30];
  	char c='a';
  	obstacles obst;
  	robot_pos robot;

  	if((p=fopen("warehouse.mp","r"))==NULL)
  	{
  		//printf("cannot open obstacle files. using the environment without obstacles now\n");
  		//return;
  		goto out2;
  	}


  	while(c!=EOF)
  	{
  		//fgets(temp,30,p);
  		fscanf(p,"%lf %lf %lf %lf",&obst.x0,&obst.y0,&obst.x1,&obst.y1);
  		c=fgetc(p);
  		/*
  		if(obst.x0>obst.x1)
  		{
  			double temp_swap=obst.x1;
  			obst.x1=obst.x0;
  			obst.x0=temp_swap;
  		}
  		if(obst.y0>obst.y1)
  		{
  		  	double temp_swap=obst.y1;
  		  	obst.y1=obst.y0;
  		  	obst.y0=temp_swap;
  		}
  		*/
  		obsts.push_back(obst);
  	}
/*
  	for(int i=0;i<obsts.size();i++)
  	{
  		std::cout<<obsts.at(i).x0<<" "<<obsts.at(i).y0<<" "<<obsts.at(i).x1<<" "<<obsts.at(i).y1<<std::endl;
  	}
*/
out2:  	fclose(p);

  	if((p=fopen("path.dat","r"))==NULL)
  	{
  	  	printf("no solution file\n");
  	  	exit(0);

  	}

  	char c1='b';
   while(c1!=EOF)
   {

	   if(agent_num==1) fscanf(p,"%lf %lf",&robot.x1,&robot.y1);
	   else if(agent_num==2) fscanf(p,"%lf %lf %lf %lf",&robot.x1,&robot.y1,&robot.x2,&robot.y2);
	   else if(agent_num==3) fscanf(p,"%lf %lf %lf %lf %lf %lf",&robot.x1,&robot.y1,&robot.x2,&robot.y2,&robot.x3,&robot.y3);
	   else if(agent_num==4) fscanf(p,"%lf %lf %lf %lf %lf %lf %lf %lf",&robot.x1,&robot.y1,&robot.x2,&robot.y2,&robot.x3,&robot.y3,&robot.x4,&robot.y4);
	   else if(agent_num==5) fscanf(p,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",&robot.x1,&robot.y1,&robot.x2,&robot.y2,&robot.x3,&robot.y3,&robot.x4,&robot.y4,&robot.x5,&robot.y5);


  	   fgetc(p);
  	   c1=fgetc(p);

  	  	path.push_back(robot);
   }

   fclose(p);
}

void drawMap(double env_width,double env_height,int agent_num)
{
	XInitThreads();
	int gd=DETECT,gm;
	initgraph(&gd,&gm,NULL);


	loadData(agent_num);

	drawObsts(env_width,env_height);

	drawPath(path,env_width,env_height,agent_num);

	getchar();

	closegraph();
}


#endif /* DRAW_H_ */
