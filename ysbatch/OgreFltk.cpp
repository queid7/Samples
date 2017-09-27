//
	 
#include "stdafx.h"
#include "../../MainLib/MainLib.h"

#include "../../MainLib/OgreFltk/FltkAddon.h"
#include "../../MainLib/OgreFltk/FlLayout.h"
#include "../../MainLib/OgreFltk/FlChoice.h"



#include <math.h>
#include "../../BaseLib/motion/Motion.h"
#include "../../BaseLib/utility/operatorString.h"
#include "../../MainLib/OgreFltk/MotionPanel.h"
#include "../../MainLib/OgreFltk/fastCapture.h"
#include "../../MainLib/OgreFltk/MotionManager.h"
#include "../../MainLib/OgreFltk/FltkRenderer.h"
#include "../../MainLib/OgreFltk/Line3D.h"

#include <Ogre.h>
#include <Fl/Fl_Tile.H>

#include "OgreFltk.h"
MotionPanel* g_motionPanel=NULL;
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);

static FastCapture* mFastCapture=NULL;

FlLayout* createRigidBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, int argc, char* argv[]);
FlLayout* createRigidBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, int argc, char* argv[], const char* a, const char* b);
void RigidBodyWin_firstInit(FlLayout* l);

//FlLayout* g_testWin;
//FlLayout* g_testWin2;
//FlLayout* g_testWin3;
MainWin::MainWin(int w, int h, int rw, int rh, OgreRenderer * pOgreRenderer, int argc, char* argv[], const char* title) 
	: Fl_Window(w, h, title)		
{
	m_tile=new Fl_Tile (0,0,w,h);
	int RENDERER_WIDTH=rw;
	int RENDERER_HEIGHT=rh;
	int WIDTH=w;
	int HEIGHT=h;
	
	m_motionPanel=new MotionPanel(0,RENDERER_HEIGHT,WIDTH, HEIGHT-RENDERER_HEIGHT);
	m_Renderer=new FltkRenderer(0,0,RENDERER_WIDTH,RENDERER_HEIGHT,pOgreRenderer);
	m_Renderer->end();
	
	int ww, hh;

	ww=WIDTH-RENDERER_WIDTH;
	hh=RENDERER_HEIGHT;

	std::vector<std::string> name, path;
	name.push_back("driverDynamicMuscle.lua");                	path.push_back("../Samples/ysscripts/samples/");
	
	int numWin = name.size();
	m_pParentWin=new FlChoiceWins(RENDERER_WIDTH, 0, ww, hh, numWin);
	for(int i=0; i<numWin; ++i)
	{
		FlLayout* testwin = createRigidBodyWin(0, 0, ww, hh, *m_motionPanel, *m_Renderer , argc, argv, name[i].c_str(), path[i].c_str());
		_testwins.push_back(testwin);
		m_pParentWin->window(i, name[i].c_str(), testwin);
	}
	//m_pParentWin->window(curWin++, "tools", createTools(ww,hh));

	m_tile->end();
	end();		

	resizable(this);

	g_motionPanel=m_motionPanel;
}

#include "../../ClassificationLib/motion/SegmentationWin.h"
#include "../../ClassificationLib/motion/ClassifyWin.h"
#include "../../ClassificationLib/motion/ClassifyBySVM.h"
#include "../../ClassificationLib/motion/ClassifyBySVMs.h"
#include "../../ClassificationLib/motion/ClassifyPoseBySVMs.h"
#include "../../ClassificationLib/motion/ParamSpaceWin.h"
#include "../../ClassificationLib/motion/PoseTrajWin.h"

extern ConfigTable config;
FlChoiceWins* MainWin::createTools(int ww,int hh)
{
	int RENDERER_WIDTH=config.GetInt("renderer_width");

	FlChoiceWins* unused=new FlChoiceWins(RENDERER_WIDTH, 0, ww, hh, 5, "SubMenu");
	{
		int curWin=0;
		//unused->window(curWin++, "Segmentation_Old", new SegmentationWin(0,0,ww,hh,m_motionPanel));
		unused->window(curWin++, "Clustering", new ClassifyWin2(0,0,ww,hh,m_motionPanel));
		//unused->window(curWin++, "Clustering_Old", new ClusteringWin(0,0,ww,hh,m_motionPanel));
		//unused->window(curWin++, "ClassificationSVM", new ClassifyBySVM(0,0,ww,hh, m_motionPanel));	
		unused->window(curWin++, "ClassificationSVMs", new ClassifyBySVMs(0,0,ww,hh, m_motionPanel));	
		unused->window(curWin++, "Show GRC", new MotionClustering::ShowGRC(0,0,ww,hh,*m_motionPanel));	
		unused->window(curWin++, "ShowPoses", new PoseTrajWin(0,0,ww,hh, m_motionPanel));	
		unused->window(curWin++, "ShowSegmentation", new ShowSegmentsWin(0,0,ww,hh, m_motionPanel));	
		unused->end();		
	}
	return unused;
}

MainWin::~MainWin()
{
	for(size_t i=0; i<_testwins.size(); ++i)
	{
		m_tile->remove(_testwins[i]);
		delete _testwins[i];
	}
}

bool MainWin::frameStarted(const Ogre::FrameEvent& evt)
{
	return true;
}

// for window capture
bool MainWin::frameEnded(const Ogre::FrameEvent& evt)
{
#ifndef NO_GUI
	if(mFastCapture)
	{
		int width=w();
		int height=h();

		//BEGIN_TIMER(screenshot);
		//for(int i=0; i<10; i++)
		mFastCapture->setCursorPos(RE::getGlobalMouseX(), RE::getGlobalMouseY());
		mFastCapture->Screenshot(this, width, height);
		//f->Screenshot(width, height);
		//END_TIMER(screenshot);

		std::cout << "screenshot" <<std::endl;
	}
#endif
	return true;
}
int MainWin::handle(int ev)
{
#ifndef NO_GUI
	if(ev==FL_MOVE || ev==FL_DRAG)
	{
		RE::setGlobalMousePos(Fl::event_x() ,Fl::event_y());

	}
	else if(ev==FL_KEYBOARD)
	{
		if(Fl::event_ctrl() && (Fl::event_state()&FL_SHIFT) && Fl::event_key()=='c' )
		{
			if(!mFastCapture)
			{
				mFastCapture=new FastCaptureDirect("../dump/mainwin.dat");
				printf("Mainwin capture started\n");
				RE::renderer().mbScreenshot=true;
			}
			else
			{
				delete mFastCapture;
				mFastCapture=NULL;
				RE::renderer().mbScreenshot=false;
			}
			return 1;
		}
	}

	return Fl_Window::handle(ev);
#else 
  return 1;

#endif
}

//#ifdef _MSC_VER
//int _tmain(int argc, char* argv[])
//#else
int main(int argc, char* argv[])
//#endif
{

	//srand((unsigned)time( NULL ));
	
	try {

		if (!Fl::visual(FL_DOUBLE|FL_INDEX))
			printf("Xdbe not supported, faking double buffer with pixmaps.\n"); 
		Fl::scheme("plastic");

		OgreRenderer renderer;

		{
			int RENDERER_WIDTH=config.GetInt("renderer_width");
			int RENDERER_HEIGHT=config.GetInt("renderer_height");
			int WIDTH=RENDERER_WIDTH+180;
			int HEIGHT=RENDERER_HEIGHT+180;
			MainWin win(WIDTH, HEIGHT, RENDERER_WIDTH, RENDERER_HEIGHT,&renderer, argc, argv, "KickBoxer");

			// ys
			win.hotspot(0,0);

			win.show();

			win.m_Renderer->firstInit(&win);
			renderer.mRoot->addFrameListener(&win);

			try
			{
				{
					LUAwrapper L;
					Register_baselib(L.L);
					Register_mainlib(L.L);

					//L.dofile("../Resource/scripts/loadBG_default.lua");
					L.dofile("../Resource/scripts/loadBG_muscle.lua");
				}
				for(size_t i=0; i<win._testwins.size(); ++i)
					RigidBodyWin_firstInit(win._testwins[i]);

				win.m_Renderer->loop(win);	
			}
			catch (char* error)
			{
				Msg::msgBox("%s", error);
				assert(false);
			}
			catch(std::exception& e)
			{
				Msg::msgBox("c++ error : %s", e.what());
				ASSERT(0);
			}
			catch(...)
			{
				Msg::msgBox("some error");
				ASSERT(0);
			}

		}
	} 
	catch( Ogre::Exception& e ) 
	{
		Msg::msgBox(e.getFullDescription().c_str());

	}	
	catch(char * error)
	{
		Msg::msgBox("%s", error);
	}
	catch(const char* error)
	{
		Msg::msgBox("%s", error);
	}
	catch(std::exception& e)
	{
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch(...)
	{
		Msg::msgBox("some error");
		ASSERT(0);
	}

	return 0;
}

