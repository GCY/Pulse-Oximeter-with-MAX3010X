#include "connectargsdlg.h"

IMPLEMENT_CLASS(ConnectArgsDialog,wxDialog)

ConnectArgsDialog::ConnectArgsDialog():root("/dev/")
{
   Init();
}

ConnectArgsDialog::ConnectArgsDialog(wxWindow *parent,wxWindowID id,const wxString &caption,const wxPoint &pos,const wxSize &size,long style):root("/dev/")
{
   Create(parent,id,caption,pos,size,style);
   Init();
}

inline void ConnectArgsDialog::Init()
{
}

bool ConnectArgsDialog::Create(wxWindow *parent,wxWindowID id,const wxString &caption,const wxPoint &pos,const wxSize &size,long style)
{
   SetExtraStyle(wxWS_EX_BLOCK_EVENTS | wxDIALOG_EX_CONTEXTHELP);

   if(!wxDialog::Create(parent,id,caption,pos,size,style)){
      return false;
   }

   CreateControls();

   GetSizer()->Fit(this);
   GetSizer()->SetSizeHints(this);

   Center();

   return true;
}

void ConnectArgsDialog::CreateControls()
{
   wxBoxSizer *top = new wxBoxSizer(wxVERTICAL);
   this->SetSizer(top);

   wxBoxSizer *box = new wxBoxSizer(wxHORIZONTAL);
   top->Add(box,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,5);

   wxArrayString paths;

   DIR *dp;
   struct dirent *dirp;
   if((dp  = opendir(root.mb_str())) == NULL) {
   }
   while ((dirp = readdir(dp)) != NULL) {
      std::string filename(dirp->d_name);
      if(filename.find(std::string("cu.")) != std::string::npos){
	 paths.Add(dirp->d_name);
	 continue;
      }
      if(filename.find(std::string("tty.")) != std::string::npos){
	 paths.Add(dirp->d_name);
	 continue;
      }
   }
   closedir(dp);   
   
/*
   struct udev *udev;
   struct udev_enumerate *enumerate;
   struct udev_list_entry *devices,*dev_list_entry;
   struct udev_device *dev;

   udev = udev_new();
   enumerate = udev_enumerate_new(udev);
   udev_enumerate_add_match_subsystem(enumerate,"tty");
   udev_enumerate_scan_devices(enumerate);
   devices = udev_enumerate_get_list_entry(enumerate);

   udev_list_entry_foreach(dev_list_entry,devices){
      const char *path;

      path = udev_list_entry_get_name(dev_list_entry);
      dev = udev_device_new_from_syspath(udev,path);

      paths.Add(wxT(udev_device_get_devnode(dev)));

      dev = udev_device_get_parent_with_subsystem_devtype(dev,"usb","usb_device");

      if(!dev){
	 paths.RemoveAt(paths.GetCount() - 1);
	 break;
      }

      udev_device_unref(dev);
   }
   udev_enumerate_unref(enumerate);
   udev_unref(udev);
*/
   device_path = new wxChoice(this,wxID_ANY,wxDefaultPosition,wxDefaultSize,paths);
   box->Add(device_path,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,5);

   wxArrayString speed;
   speed.Add(wxT("300"));
   speed.Add(wxT("1200"));
   speed.Add(wxT("2400"));
   speed.Add(wxT("4800"));
   speed.Add(wxT("9600"));
   speed.Add(wxT("19200"));
   speed.Add(wxT("38400"));
   speed.Add(wxT("57600"));
   speed.Add(wxT("115200"));

   baud_rate = new wxChoice(this,wxID_ANY,wxDefaultPosition,wxDefaultSize,speed);
   box->Add(baud_rate,0,wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,5);

   wxBoxSizer *ResetOkCancelBox = new wxBoxSizer(wxHORIZONTAL);
   top->Add(ResetOkCancelBox,0,wxALIGN_CENTER_HORIZONTAL | wxALL,5);

   wxButton *ok = new wxButton(this,wxID_OK,wxT("&Ok"),wxDefaultPosition,wxDefaultSize,0);
   ResetOkCancelBox->Add(ok,0,wxALIGN_CENTER_VERTICAL | wxALL,5);

   wxButton *cancel = new wxButton(this,wxID_CANCEL,wxT("&Cancel"),wxDefaultPosition,wxDefaultSize,0);
   ResetOkCancelBox->Add(cancel,0,wxALIGN_CENTER_VERTICAL | wxALL,5);
}

bool ConnectArgsDialog::TransferDataToWindow()
{
   return true;
}

bool ConnectArgsDialog::TransferDataFromWindow()
{
   return true;
}
