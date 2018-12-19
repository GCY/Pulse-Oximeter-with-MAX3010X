#ifndef __CONNECT_ARGS_DIALOG__
#define __CONNECT_ARGS_DIALOG__
#include <wx/wx.h>
#include <sys/types.h>
#include <sys/stat.h> 
#include <dirent.h>

class ConnectArgsDialog:public wxDialog
{
   public:
      ConnectArgsDialog();

      ConnectArgsDialog(wxWindow *parent,
	    wxWindowID id = wxID_ANY,
	    const wxString &caption = wxT("Connect Args"),
	    const wxPoint &pos = wxDefaultPosition,
	    const wxSize &size = wxDefaultSize,
	    long style = wxCAPTION | wxRESIZE_BORDER | wxSYSTEM_MENU);

      bool Create(wxWindow *parent,
	    wxWindowID id = wxID_ANY,
	    const wxString &caption = wxT("Connect Args"),
	    const wxPoint &pos = wxDefaultPosition,
	    const wxSize &size = wxDefaultSize,
	    long style = wxCAPTION | wxRESIZE_BORDER | wxSYSTEM_MENU);

      void Init();
      void CreateControls();

      bool TransferDataToWindow();
      bool TransferDataFromWindow();

      wxString GetDevicePath(){return root + device_path->GetString(device_path->GetCurrentSelection());}
      wxString GetBaudRate(){return baud_rate->GetString(baud_rate->GetCurrentSelection());}

   private:
      wxChoice *device_path;
      wxChoice *baud_rate;

      wxString root;

      DECLARE_CLASS(ConnectArgsDialog)
};

#endif
