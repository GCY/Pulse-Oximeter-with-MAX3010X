#include <wx/wx.h>
#include <wx/filename.h>
#include <wx/tokenzr.h>
#include <wx/valnum.h>
#include <wx/thread.h>

#include "mathplot.h"
#include "connectargsdlg.h"
#include "serialport.h"

#include <fstream>
#include <cmath>
#include <cstdlib>
#include <ctime> 
#include <iostream>
#include <chrono>   

#define NIRS

enum{
   ID_EXIT = 200,
   ID_RECORD,
   ID_VCP,
   ID_CLEAR_ALL_PLOT,
   ID_CONNECT_DEVICE
};

enum{
   EVT_SERIAL_PORT_READ = 625,
   EVT_REFRESH_PLOT
};

class Thread;

class App:public wxApp
{
   public:
      bool OnInit();
};

class Frame:public wxFrame
{
   public:
      Frame(const wxString&);
      ~Frame();

      void CreateUI();

      void OnRecord(wxCommandEvent&);
      void OnClearAllPlot(wxCommandEvent&);
      void OnKeyDown(wxKeyEvent& event);
      void OnConnectDevice(wxCommandEvent&);
      void OnExit(wxCommandEvent&);

      void ClearAllData();

      void OnFit(wxCommandEvent&);

      void OnThreadEvent(wxThreadEvent &event);

   private:

      wxMenu *vcp;
      wxThread *thread;

      SerialPort serial;     
      bool is_open;

      bool run_flag;

      static const wxString PeakStr;
      static const wxString AbsopStr;

      mpWindow *channel1_plot;
      mpWindow *channel2_plot;  

      mpFXYVector* channel1_layer;
      std::vector<double> channel1_layer_x, channel1_layer_y;

      mpFXYVector* channel2_layer;
      std::vector<double> channel2_layer_x, channel2_layer_y;    

      clock_t record_start_time;
      bool record_flag;
      std::fstream record;

      DECLARE_EVENT_TABLE();
};

const wxString Frame::PeakStr = wxT("Peak");
const wxString Frame::AbsopStr = wxT("Absop");

class Thread:public wxThread
{
   public:
      Thread(Frame*,wxEvtHandler*);

      void* Entry();

   private:
      Frame *frame;
      wxEvtHandler *handler;
};

IMPLEMENT_APP(App)
DECLARE_APP(App)
   BEGIN_EVENT_TABLE(Frame,wxFrame)
   EVT_MENU(ID_EXIT,Frame::OnExit)
   EVT_MENU(ID_CLEAR_ALL_PLOT,Frame::OnClearAllPlot)
   EVT_MENU(ID_RECORD,Frame::OnRecord)
   EVT_MENU(ID_CONNECT_DEVICE,Frame::OnConnectDevice)
   EVT_MENU(mpID_FIT, Frame::OnFit)
   EVT_THREAD(wxID_ANY, Frame::OnThreadEvent)
END_EVENT_TABLE()

bool App::OnInit()
{
   Frame *frame = new Frame(wxT("Pulse Oximeter Monitor"));

   frame->Show(true);

   return true;
}

Frame::Frame(const wxString &title):wxFrame(NULL,wxID_ANY,title,wxDefaultPosition,wxSize(1050,700),wxMINIMIZE_BOX | wxCLOSE_BOX | wxCAPTION | wxSYSTEM_MENU)
{
   run_flag = true;
   record_flag = false;

   CreateUI();

   Bind(wxEVT_CHAR_HOOK,&Frame::OnKeyDown,this);

   channel1_layer = new mpFXYVector(wxT("IR 940nm"),mpALIGN_NE);
   channel1_layer->ShowName(false);
   channel1_layer->SetContinuity(true);
   wxPen vectorpen1_1(wxColour(255,205,205), 3, wxPENSTYLE_SOLID);
   channel1_layer->SetPen(vectorpen1_1);
   channel1_layer->SetDrawOutsideMargins(false);

   wxFont graphFont(11, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
   mpScaleX* xaxis = new mpScaleX(wxT("Time(index)"), mpALIGN_BOTTOM, true, mpX_NORMAL);
   mpScaleY* yaxis = new mpScaleY(wxT("ADC"), mpALIGN_LEFT, true);
   xaxis->SetFont(graphFont);
   yaxis->SetFont(graphFont);
   xaxis->SetDrawOutsideMargins(false);
   yaxis->SetDrawOutsideMargins(false);
   channel1_plot->SetMargins(30, 30, 50, 100);
   channel1_plot->AddLayer(     xaxis );
   channel1_plot->AddLayer(     yaxis );
   channel1_plot->AddLayer(     channel1_layer );

   mpInfoLegend* leg;
   channel1_plot->AddLayer( leg = new mpInfoLegend(wxRect(200,20,40,40), wxTRANSPARENT_BRUSH));
   leg->SetVisible(true);

   channel1_plot->EnableDoubleBuffer(true);
   channel1_plot->SetMPScrollbars(false);
   channel1_plot->Fit();

   channel2_layer = new mpFXYVector(wxT("RED 660nm"),mpALIGN_NE);
   channel2_layer->ShowName(false);
   channel2_layer->SetContinuity(true);
   wxPen vectorpen2_1(wxColour(255,0,0), 3, wxPENSTYLE_SOLID);
   channel2_layer->SetPen(vectorpen2_1);
   channel2_layer->SetDrawOutsideMargins(false);

   xaxis = new mpScaleX(wxT("Time(index)"), mpALIGN_BOTTOM, true, mpX_NORMAL);
   yaxis = new mpScaleY(wxT("ADC"), mpALIGN_LEFT, true);
   xaxis->SetFont(graphFont);
   yaxis->SetFont(graphFont);
   xaxis->SetDrawOutsideMargins(false);
   yaxis->SetDrawOutsideMargins(false);
   channel2_plot->SetMargins(30, 30, 50, 100);
   channel2_plot->AddLayer(     xaxis );
   channel2_plot->AddLayer(     yaxis );
   channel2_plot->AddLayer(     channel2_layer );

   channel2_plot->AddLayer( leg = new mpInfoLegend(wxRect(200,20,40,40), wxTRANSPARENT_BRUSH));
   leg->SetVisible(true);

   channel2_plot->EnableDoubleBuffer(true);
   channel2_plot->SetMPScrollbars(false);
   channel2_plot->Fit();

   std::srand(std::time(nullptr));


   thread = new Thread(this,this);
   thread->Create();
   thread->Run();
}

Frame::~Frame()
{
   if(record.is_open()){
      record.close();
   }
   if(thread){
      thread->Delete();
      thread = NULL;
   }
}

void Frame::CreateUI()
{
   wxMenu *file = new wxMenu;
   file->Append(ID_RECORD,wxT("R&cord\tAlt-r"),wxT("Record"));
   file->Append(ID_EXIT,wxT("E&xit\tAlt-e"),wxT("Exit"));

   vcp = new wxMenu();
   wxMenu *tools = new wxMenu;
   tools->Append(ID_CONNECT_DEVICE,wxT("V&CP\tAlt-v"),wxT("Virtual COM Port"));

   wxMenu *view = new wxMenu;
   view->Append(mpID_FIT,wxT("F&it\tAlt-f"),wxT("Fit"));
   view->Append(ID_CLEAR_ALL_PLOT,wxT("C&lear Plot\tAlt-c"),wxT("Clear All Plot"));

   wxMenuBar *bar = new wxMenuBar;

   bar->Append(file,wxT("File"));
   bar->Append(tools,wxT("Tools"));
   bar->Append(view,wxT("View"));
   SetMenuBar(bar);

   channel1_plot = new mpWindow( this, -1, wxPoint(15,10), wxSize(1020,300), wxBORDER_SUNKEN );
   channel2_plot = new mpWindow( this, -1, wxPoint(15,330), wxSize(1024,300), wxBORDER_SUNKEN );

   CreateStatusBar(1);
   SetStatusText(wxT("Pulse Oximeter Monitor"));   
}

void Frame::OnFit( wxCommandEvent &WXUNUSED(event) )
{
   channel1_plot->Fit();
   channel2_plot->Fit();
}

void Frame::OnClearAllPlot(wxCommandEvent &event)
{
   ClearAllData();
}

void Frame::ClearAllData()
{
   channel1_plot->DelAllLayers(true);
   channel2_plot->DelAllLayers(true);

   channel1_layer_x.clear();
   channel1_layer_y.clear();

}

void Frame::OnRecord(wxCommandEvent &event)
{

   wxFileDialog fileDialog(this, _("CSV"), wxT(""), wxT("data"), wxT("CSV (*.csv)|.csv"), wxFD_SAVE);
   if(fileDialog.ShowModal() == wxID_OK) {
      wxFileName namePath(fileDialog.GetPath());
      record.open(fileDialog.GetPath().mb_str(),std::fstream::out);
      record_flag = true;
      record_start_time = clock();
   }
   else{
      return;
   }

   event.Skip();
}

void Frame::OnConnectDevice(wxCommandEvent &event)
{
   ConnectArgsDialog dlg(this,wxID_ANY,wxT("Connect"),wxDefaultPosition,wxDefaultSize,wxDEFAULT_DIALOG_STYLE);

   if(dlg.ShowModal() == wxID_OK){
#ifdef _WIN_
      is_open = serial.Open(dlg.GetDevicePath().wc_str());
#elif _MAC_
      is_open = serial.Open(dlg.GetDevicePath().c_str());
#endif
      serial.SetBaudRate(wxAtoi(dlg.GetBaudRate()));
      serial.SetParity(8,1,'N');

      run_flag = true;

      if(is_open){
	 unsigned char gid[4] = "DID";
	 serial.Write(gid,4);

	 unsigned char sms[4] = "CHK";
	 serial.Write(sms,4);  

	 unsigned char ok[3] = "OK";
	 serial.Write(ok,3);  	 
      }
      else{
	 wxMessageBox(wxT("Serial Port Error!"),wxT("Error!"));
      }

   }
}

void Frame::OnKeyDown(wxKeyEvent& event)
{
   if(event.GetKeyCode() == 32){
      run_flag ^= true;
   }
   event.Skip();
}

void Frame::OnExit(wxCommandEvent &event)
{
   Close();
}

clock_t start_time = std::clock();
clock_t spo2_time = std::clock();
double spo2_sum = 0;
double last_spo2 = 0;
unsigned spo2_num = 0;
double mspo2_sum = 0;
int mspo2_num = 0;
int ttt = 0;

int32_t ir_ac = 0;
int32_t red_ac = 0;
int32_t ac_num = 0;
int32_t ir_dc_sum = 0;
int32_t red_dc_sum = 0;
int32_t dc_num = 0;

std::chrono::steady_clock::time_point sampling_time = std::chrono::steady_clock::now();
unsigned int sampling_rate = 0;

void Frame::OnThreadEvent(wxThreadEvent &event)
{

   const size_t MAX_POINT = 1000;   

   if(event.GetInt() == EVT_SERIAL_PORT_READ && is_open && run_flag){

      unsigned char buffer[3000]={0};
      int length = serial.Read(buffer);

      uint32_t irACValueSqSum = 0;
      uint32_t redACValueSqSum = 0;
      uint16_t samplesRecorded = 0;
      uint16_t redLEDCurrent = 0;

      if(length != -1){
	 wxStringTokenizer tokenizer1(buffer,"R");
	 if(tokenizer1.CountTokens() == 0){
	    return ;
	 }
	 tokenizer1.GetNextToken();

	 while(tokenizer1.HasMoreTokens()){
	    wxString split = tokenizer1.GetNextToken();
	    //wxLogDebug(split);

	    ++sampling_rate;
	    if(std::chrono::steady_clock::duration(std::chrono::steady_clock::now() - sampling_time).count() > std::chrono::steady_clock::period::den){
	       wxLogDebug(wxT("SR: %d"),sampling_rate);
	       sampling_rate = 0;
	       sampling_time = std::chrono::steady_clock::now();
	    }

	    wxStringTokenizer tokenizer2(split,",");

	    if(tokenizer2.CountTokens() < 10){
	       continue;
	    }

	    if(record_flag){
	       record << split << "," << last_spo2 << std::endl;
	       if(clock() - record_start_time > 60*CLOCKS_PER_SEC){
		  record_flag = false;
		  record.close();
		  wxMessageBox(wxT("Record 1 minumte data!"),wxT("DONE!"));
	       }
	    }

	    int token_index = 0;

	    while(tokenizer2.HasMoreTokens()){
	       wxString str = tokenizer2.GetNextToken();   
	       if(token_index == 1){/*
				       channel1_layer_x.push_back( ttt );
		  //channel1_layer_y.push_back(fir1.filter(wxAtoi(str)));
		  channel1_layer_y.push_back(wxAtoi(str));
		  if (channel1_layer_x.size() > MAX_POINT && channel1_layer_y.size() > MAX_POINT){
		  channel1_layer_x.erase(channel1_layer_x.begin());
		  channel1_layer_y.erase(channel1_layer_y.begin());
		  } */
	       }
	       if(token_index == 2){/*
				       channel2_layer_x.push_back( ttt );
		  //channel2_layer_y.push_back(fir2.filter(wxAtoi(str)));
		  channel2_layer_y.push_back(wxAtoi(str));
		  if (channel2_layer_x.size() > MAX_POINT && channel2_layer_y.size() > MAX_POINT){
		  channel2_layer_x.erase(channel2_layer_x.begin());
		  channel2_layer_y.erase(channel2_layer_y.begin());
		  } 
		  ++ttt;	*/	  
	       }
	       if(token_index == 3){
		  ir_dc_sum += wxAtoi(str);
	       }
	       if(token_index == 4){
		  red_dc_sum += wxAtoi(str);
		  ++dc_num;
	       }
	       if(token_index == 5){
		  ir_ac += wxAtoi(str) * wxAtoi(str); 

		  channel1_layer_x.push_back( ttt );
		  //channel1_layer_y.push_back(fir1.filter(wxAtoi(str)));
		  channel1_layer_y.push_back(wxAtoi(str));
		  if (channel1_layer_x.size() > MAX_POINT && channel1_layer_y.size() > MAX_POINT){
		     channel1_layer_x.erase(channel1_layer_x.begin());
		     channel1_layer_y.erase(channel1_layer_y.begin());
		  } 
	       }
	       if(token_index == 6){
		  red_ac += wxAtoi(str) * wxAtoi(str);


		  channel2_layer_x.push_back( ttt );
		  //channel2_layer_y.push_back(fir2.filter(wxAtoi(str)));
		  channel2_layer_y.push_back(wxAtoi(str));
		  if (channel2_layer_x.size() > MAX_POINT && channel2_layer_y.size() > MAX_POINT){
		     channel2_layer_x.erase(channel2_layer_x.begin());
		     channel2_layer_y.erase(channel2_layer_y.begin());
		  } 

		  ++ttt;
		  ++ac_num;
	       }
	       if(token_index == 7){
		  irACValueSqSum = wxAtoi(str);
	       }
	       if(token_index == 8){
		  redACValueSqSum = wxAtoi(str);
	       }
	       if(token_index == 9){
		  redLEDCurrent = wxAtoi(str);
	       }
	       if(token_index == 11){
		  samplesRecorded = wxAtoi(str);
		  if(samplesRecorded != 0){
		     
		     float red_log_rms = log( sqrt(redACValueSqSum/samplesRecorded) );
		     float ir_log_rms = log( sqrt(irACValueSqSum/samplesRecorded) );
		     float ratioRMS = 0.0f;
		     if(red_log_rms != 0.0f && ir_log_rms != 0.0f){
			ratioRMS = red_log_rms / ir_log_rms;
		     }

		     //wxLogDebug(wxT("%f%"),110.0f - 14.0f * ratioRMS);
		     spo2_sum += 110.0f - 14.0f * ratioRMS;
		     //spo2_sum += (-45.060* (ratioRMS*ratioRMS) ) + (30.354 * ratioRMS) + 94.845;

		     ++spo2_num;

		     if(std::clock() - spo2_time > CLOCKS_PER_SEC){
			last_spo2 = spo2_sum / spo2_num;

			mspo2_sum += last_spo2;
			++mspo2_num;

			int32_t ir_dc = (ir_dc_sum) / dc_num;
			int32_t red_dc = (red_dc_sum) / dc_num;

			/*
			ir_ac_rms = sqrt(ir_ac/ac_num); // sqrt(irACValueSqSum/samplesRecorded)
			red_ac_rms = sqrt(red_ac/ac_num); // sqrt(redACValueSqSum/samplesRecorded)
			*/

			wxString spo2;
			spo2.Printf(wxT("SPo2 : %f , mSPo2 : %f , redLEDCurrent : %d"), last_spo2, mspo2_sum / mspo2_num, redLEDCurrent);
			SetStatusText(spo2);

			if(mspo2_num > 30){
			   mspo2_num = 0;
			   mspo2_sum = 0;
			}			

			dc_num = 0;
			ir_dc_sum = 0;
			red_dc_sum = 0;

			ac_num = 0;
			ir_ac = 0;
			red_ac = 0;

			spo2_sum = 0;
			spo2_num = 0;
			spo2_time = std::clock();
		     }

		  }
	       }
	       ++token_index;
	    }
	 }

      }  
   }

   if(event.GetInt() == EVT_REFRESH_PLOT && is_open && run_flag){
      channel1_layer->SetData(channel1_layer_x, channel1_layer_y);
      channel2_layer->SetData(channel2_layer_x, channel2_layer_y);
      channel1_plot->Fit();
      channel2_plot->Fit();      
   }

}

Thread::Thread(Frame *parent,wxEvtHandler *evt):wxThread(wxTHREAD_DETACHED),handler(evt)
{
   frame = parent;
}

clock_t refresh_last = std::clock();
clock_t read_last = std::clock();
void* Thread::Entry()
{

   const clock_t READ_RATE = 500;
   const clock_t FRAME_RATE = 20;   

   while(!TestDestroy()){
      if(clock() - read_last > (CLOCKS_PER_SEC/READ_RATE)){
	 wxThreadEvent *evt = new wxThreadEvent(wxEVT_THREAD);
	 evt->SetInt(EVT_SERIAL_PORT_READ);
	 handler->QueueEvent(evt);
	 read_last = clock();
      }
      if(std::clock() - refresh_last > (CLOCKS_PER_SEC/FRAME_RATE)){
	 wxThreadEvent *evt = new wxThreadEvent(wxEVT_THREAD);
	 evt->SetInt(EVT_REFRESH_PLOT);
	 handler->QueueEvent(evt);
	 refresh_last = std::clock();
      }
   }

   return NULL;
}

