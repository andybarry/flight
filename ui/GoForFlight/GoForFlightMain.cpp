/***************************************************************
 * Name:      GoForFlightMain.cpp
 * Purpose:   Code for Application Frame
 * Author:    Andy Barry (abarry@gmail.com)
 * Created:   2015-02-03
 * Copyright: Andy Barry (abarry.org)
 * License:
 **************************************************************/

#include "GoForFlightMain.h"
#include <wx/msgdlg.h>
#include "StatusHandler.h"

//(*InternalHeaders(GoForFlightFrame)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
//*)

//helper functions
enum wxbuildinfoformat {
    short_f, long_f };

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f )
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

//(*IdInit(GoForFlightFrame)
const long GoForFlightFrame::ID_STATICTEXT2 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT30 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT31 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT20 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT5 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT21 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT22 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT3 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT14 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT7 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT15 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT8 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT16 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT9 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT36 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT6 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT4 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT34 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT32 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT33 = wxNewId();
const long GoForFlightFrame::ID_CHECKBOX1 = wxNewId();
const long GoForFlightFrame::ID_CHECKBOX2 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT1 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT10 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT17 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT11 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT18 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT12 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT19 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT13 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT23 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT24 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT25 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT26 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT27 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT28 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT29 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT38 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT37 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT39 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT40 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT41 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT42 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT43 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT35 = wxNewId();
const long GoForFlightFrame::ID_PANEL1 = wxNewId();
const long GoForFlightFrame::idMenuQuit = wxNewId();
const long GoForFlightFrame::idMenuAbout = wxNewId();
const long GoForFlightFrame::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(GoForFlightFrame,wxFrame)
    //(*EventTable(GoForFlightFrame)
    //*)
END_EVENT_TABLE()

GoForFlightFrame::GoForFlightFrame(wxWindow* parent,wxWindowID id)
{

    //(*Initialize(GoForFlightFrame)
    wxMenuItem* MenuItem2;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxBoxSizer* BoxSizer3;
    wxMenu* Menu1;
    wxBoxSizer* BoxSizer10;
    wxBoxSizer* BoxSizer7;
    wxBoxSizer* BoxSizer11;
    wxBoxSizer* BoxSizer2;
    wxFlexGridSizer* FlexGridSizer4;
    wxBoxSizer* BoxSizer9;
    wxFlexGridSizer* FlexGridSizer3;
    wxBoxSizer* BoxSizer4;
    wxBoxSizer* BoxSizer8;
    wxBoxSizer* BoxSizer1;
    wxMenuBar* MenuBar1;
    wxMenu* Menu2;
    wxFlexGridSizer* FlexGridSizer5;
    wxBoxSizer* BoxSizer6;
    wxBoxSizer* BoxSizer5;
    wxStaticBoxSizer* StaticBoxSizer1;

    Create(parent, wxID_ANY, _("Go For Flight"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    {
    	wxIcon FrameIcon;
    	FrameIcon.CopyFromBitmap(wxBitmap(wxImage(_T("/home/abarry/realtime/ui/GoForFlight/checkbox_icon.png"))));
    	SetIcon(FrameIcon);
    }
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    Panel1 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    BoxSizer3 = new wxBoxSizer(wxVERTICAL);
    lblTimesync = new wxStaticText(Panel1, ID_STATICTEXT2, _("Timesync: Offline"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    BoxSizer3->Add(lblTimesync, 0, wxTOP|wxLEFT|wxRIGHT|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer4 = new wxBoxSizer(wxHORIZONTAL);
    BoxSizer4->Add(46,20,0, wxALL|wxFIXED_MINSIZE|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer4 = new wxFlexGridSizer(0, 2, 0, 0);
    lblTimesyncLabelLocal = new wxStaticText(Panel1, ID_STATICTEXT30, _("Local:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT30"));
    FlexGridSizer4->Add(lblTimesyncLabelLocal, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblTimesyncLocal = new wxStaticText(Panel1, ID_STATICTEXT31, _("--/--/-- | --:--:--"), wxDefaultPosition, wxSize(264,20), 0, _T("ID_STATICTEXT31"));
    FlexGridSizer4->Add(lblTimesyncLocal, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblTimesyncLabelGps = new wxStaticText(Panel1, ID_STATICTEXT20, _("GPS:"), wxDefaultPosition, wxSize(47,20), 0, _T("ID_STATICTEXT20"));
    FlexGridSizer4->Add(lblTimesyncLabelGps, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblTimesyncGps = new wxStaticText(Panel1, ID_STATICTEXT5, _("--/--/-- | --:--:--"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer4->Add(lblTimesyncGps, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblTimesyncLabelCam = new wxStaticText(Panel1, ID_STATICTEXT21, _("Cam:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT21"));
    FlexGridSizer4->Add(lblTimesyncLabelCam, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblTimesyncCam = new wxStaticText(Panel1, ID_STATICTEXT22, _("--/--/-- | --:--:--"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT22"));
    FlexGridSizer4->Add(lblTimesyncCam, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer4->Add(FlexGridSizer4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer3->Add(BoxSizer4, 0, wxBOTTOM|wxLEFT|wxRIGHT|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 0);
    lblLogging = new wxStaticText(Panel1, ID_STATICTEXT3, _("Logging: Offline"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    BoxSizer3->Add(lblLogging, 0, wxTOP|wxLEFT|wxRIGHT|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer5 = new wxBoxSizer(wxHORIZONTAL);
    BoxSizer5->Add(46,20,0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3 = new wxFlexGridSizer(0, 2, 0, 0);
    lblLogLocalLabel = new wxStaticText(Panel1, ID_STATICTEXT14, _("Local:"), wxDefaultPosition, wxSize(59,20), 0, _T("ID_STATICTEXT14"));
    FlexGridSizer3->Add(lblLogLocalLabel, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblLogLocal = new wxStaticText(Panel1, ID_STATICTEXT7, _("#--"), wxDefaultPosition, wxSize(127,20), 0, _T("ID_STATICTEXT7"));
    FlexGridSizer3->Add(lblLogLocal, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblLogGpsLabel = new wxStaticText(Panel1, ID_STATICTEXT15, _("GPS:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT15"));
    FlexGridSizer3->Add(lblLogGpsLabel, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblLogGps = new wxStaticText(Panel1, ID_STATICTEXT8, _("#--"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
    FlexGridSizer3->Add(lblLogGps, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblLogCamLabel = new wxStaticText(Panel1, ID_STATICTEXT16, _("Cam:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT16"));
    FlexGridSizer3->Add(lblLogCamLabel, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblLogCam = new wxStaticText(Panel1, ID_STATICTEXT9, _("#--"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    FlexGridSizer3->Add(lblLogCam, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer5->Add(FlexGridSizer3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer3->Add(BoxSizer5, 0, wxBOTTOM|wxLEFT|wxRIGHT|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblAirspeed = new wxStaticText(Panel1, ID_STATICTEXT36, _("Airspeed: Offline (Mean: -- / Std Dev: --)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT36"));
    BoxSizer3->Add(lblAirspeed, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblGps = new wxStaticText(Panel1, ID_STATICTEXT6, _("GPS: Offline (Sats: --)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    BoxSizer3->Add(lblGps, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblStateEsimator = new wxStaticText(Panel1, ID_STATICTEXT4, _("State Estimator: Offline"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    BoxSizer3->Add(lblStateEsimator, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblController = new wxStaticText(Panel1, ID_STATICTEXT34, _("Controller: Offline"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT34"));
    BoxSizer3->Add(lblController, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblBatteryStatus = new wxStaticText(Panel1, ID_STATICTEXT32, _("Battery Voltage: --"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT32"));
    BoxSizer3->Add(lblBatteryStatus, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblStereo = new wxStaticText(Panel1, ID_STATICTEXT33, _("Vision: Offline"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT33"));
    BoxSizer3->Add(lblStereo, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    chkGoPro = new wxCheckBox(Panel1, ID_CHECKBOX1, _("GoPro Rolling"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    chkGoPro->SetValue(false);
    BoxSizer3->Add(chkGoPro, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    chkObstacleGoPro = new wxCheckBox(Panel1, ID_CHECKBOX2, _("Obstacle GoPro Rolling"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    chkObstacleGoPro->SetValue(false);
    BoxSizer3->Add(chkObstacleGoPro, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer2->Add(BoxSizer3, 0, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer8 = new wxBoxSizer(wxVERTICAL);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxVERTICAL, Panel1, wxEmptyString);
    BoxSizer6 = new wxBoxSizer(wxHORIZONTAL);
    BoxSizer6->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    lblGoForFlight = new wxStaticText(Panel1, ID_STATICTEXT1, _("No Go"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT1"));
    wxFont lblGoForFlightFont(20,wxSWISS,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Sans"),wxFONTENCODING_DEFAULT);
    lblGoForFlight->SetFont(lblGoForFlightFont);
    BoxSizer6->Add(lblGoForFlight, 0, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer6->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer1->Add(BoxSizer6, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer8->Add(StaticBoxSizer1, 0, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    lblDiskFree = new wxStaticText(Panel1, ID_STATICTEXT10, _("Disk Free: --"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
    BoxSizer8->Add(lblDiskFree, 0, wxTOP|wxLEFT|wxRIGHT|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer7 = new wxBoxSizer(wxHORIZONTAL);
    BoxSizer7->Add(46,20,0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2 = new wxFlexGridSizer(0, 2, 0, 0);
    lblDiskFreeLabelLocal = new wxStaticText(Panel1, ID_STATICTEXT17, _("Local:"), wxDefaultPosition, wxSize(59,20), 0, _T("ID_STATICTEXT17"));
    FlexGridSizer2->Add(lblDiskFreeLabelLocal, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblDiskFreeLocal = new wxStaticText(Panel1, ID_STATICTEXT11, _("233.3 GB"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
    lblDiskFreeLocal->SetMinSize(wxSize(100,-1));
    FlexGridSizer2->Add(lblDiskFreeLocal, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblDiskFreeLabelGps = new wxStaticText(Panel1, ID_STATICTEXT18, _("GPS:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT18"));
    FlexGridSizer2->Add(lblDiskFreeLabelGps, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblDiskFreeGps = new wxStaticText(Panel1, ID_STATICTEXT12, _("-- GB"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT12"));
    FlexGridSizer2->Add(lblDiskFreeGps, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblDiskFreeLabelCam = new wxStaticText(Panel1, ID_STATICTEXT19, _("Cam:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT19"));
    FlexGridSizer2->Add(lblDiskFreeLabelCam, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblDiskFreeCam = new wxStaticText(Panel1, ID_STATICTEXT13, _("-- GB"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT13"));
    FlexGridSizer2->Add(lblDiskFreeCam, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer7->Add(FlexGridSizer2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer8->Add(BoxSizer7, 0, wxBOTTOM|wxLEFT|wxRIGHT|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer9 = new wxBoxSizer(wxVERTICAL);
    lblCpu = new wxStaticText(Panel1, ID_STATICTEXT23, _("CPU: --"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT23"));
    BoxSizer9->Add(lblCpu, 0, wxTOP|wxLEFT|wxRIGHT|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer10 = new wxBoxSizer(wxHORIZONTAL);
    BoxSizer10->Add(46,20,0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer1 = new wxFlexGridSizer(0, 2, 0, 0);
    lblCpuLocalLabel = new wxStaticText(Panel1, ID_STATICTEXT24, _("Local:"), wxDefaultPosition, wxSize(59,20), 0, _T("ID_STATICTEXT24"));
    FlexGridSizer1->Add(lblCpuLocalLabel, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCpuLocal = new wxStaticText(Panel1, ID_STATICTEXT25, _("-- Ghz / -- C"), wxDefaultPosition, wxSize(205,20), 0, _T("ID_STATICTEXT25"));
    FlexGridSizer1->Add(lblCpuLocal, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCpuGpsLabel = new wxStaticText(Panel1, ID_STATICTEXT26, _("GPS:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT26"));
    FlexGridSizer1->Add(lblCpuGpsLabel, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCpuGps = new wxStaticText(Panel1, ID_STATICTEXT27, _("-- Ghz / -- C"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT27"));
    FlexGridSizer1->Add(lblCpuGps, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCpuCamLabel = new wxStaticText(Panel1, ID_STATICTEXT28, _("Cam:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT28"));
    FlexGridSizer1->Add(lblCpuCamLabel, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCpuCam = new wxStaticText(Panel1, ID_STATICTEXT29, _("-- Ghz / -- C"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT29"));
    FlexGridSizer1->Add(lblCpuCam, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer10->Add(FlexGridSizer1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer9->Add(BoxSizer10, 0, wxLEFT|wxRIGHT|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    lblCodeSync = new wxStaticText(Panel1, ID_STATICTEXT38, _("Code sync: --"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT38"));
    BoxSizer9->Add(lblCodeSync, 0, wxTOP|wxLEFT|wxRIGHT|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer11 = new wxBoxSizer(wxHORIZONTAL);
    BoxSizer11->Add(46,20,0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5 = new wxFlexGridSizer(0, 2, 0, 0);
    lblCodeSyncLocalLabel = new wxStaticText(Panel1, ID_STATICTEXT37, _("Local:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT37"));
    FlexGridSizer5->Add(lblCodeSyncLocalLabel, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCodeSyncLocal = new wxStaticText(Panel1, ID_STATICTEXT39, _("--"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT39"));
    FlexGridSizer5->Add(lblCodeSyncLocal, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCodeSyncGpsLabel = new wxStaticText(Panel1, ID_STATICTEXT40, _("GPS:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT40"));
    FlexGridSizer5->Add(lblCodeSyncGpsLabel, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCodeSyncGps = new wxStaticText(Panel1, ID_STATICTEXT41, _("--"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT41"));
    FlexGridSizer5->Add(lblCodeSyncGps, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCodeSyncCamLabel = new wxStaticText(Panel1, ID_STATICTEXT42, _("Cam:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT42"));
    FlexGridSizer5->Add(lblCodeSyncCamLabel, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblCodeSyncCam = new wxStaticText(Panel1, ID_STATICTEXT43, _("--"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT43"));
    FlexGridSizer5->Add(lblCodeSyncCam, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer11->Add(FlexGridSizer5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer9->Add(BoxSizer11, 0, wxLEFT|wxRIGHT|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblDebug = new wxStaticText(Panel1, ID_STATICTEXT35, _("Debug: OK"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT35"));
    BoxSizer9->Add(lblDebug, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer8->Add(BoxSizer9, 0, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer2->Add(BoxSizer8, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel1->SetSizer(BoxSizer2);
    BoxSizer2->Fit(Panel1);
    BoxSizer2->SetSizeHints(Panel1);
    BoxSizer1->Add(Panel1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    SetSizer(BoxSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    tmrUpdateLcm.SetOwner(this, ID_TIMER1);
    tmrUpdateLcm.Start(100, false);
    BoxSizer1->Fit(this);
    BoxSizer1->SetSizeHints(this);

    Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&GoForFlightFrame::OnchkGoProClick);
    Connect(ID_CHECKBOX2,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&GoForFlightFrame::OnchkObstacleGoProClick);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&GoForFlightFrame::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&GoForFlightFrame::OnAbout);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&GoForFlightFrame::OntmrUpdateLcmTrigger);
    //*)





    log_size_handler_.SetLabel(lblLogging);

    log_size_handler_.SetTimesyncLabelMain(lblTimesync);
    log_size_handler_.SetTimesyncLabel(LOCAL, lblTimesyncLabelLocal, lblTimesyncLocal);
    log_size_handler_.SetTimesyncLabel(GPS, lblTimesyncLabelGps, lblTimesyncGps);
    log_size_handler_.SetTimesyncLabel(CAM, lblTimesyncLabelCam, lblTimesyncCam);

    log_size_handler_.SetDiskFreeLabelMain(lblDiskFree);
    log_size_handler_.SetDiskFreeLabel(LOCAL, lblDiskFreeLabelLocal, lblDiskFreeLocal);
    log_size_handler_.SetDiskFreeLabel(GPS, lblDiskFreeLabelGps, lblDiskFreeGps);
    log_size_handler_.SetDiskFreeLabel(CAM, lblDiskFreeLabelCam, lblDiskFreeCam);


    log_size_handler_.AddLabel(LOCAL, lblLogLocalLabel, lblLogLocal);
    log_size_handler_.AddLabel(GPS, lblLogGpsLabel, lblLogGps);
    log_size_handler_.AddLabel(CAM, lblLogCamLabel, lblLogCam);

    state_estimator_handler_.SetLabel(lblStateEsimator);
    gps_handler_.SetLabel(lblGps);
    controller_handler_.SetLabel(lblController);

    cpu_info_handler_.SetLabel(lblCpu);
    cpu_info_handler_.AddLabel(LOCAL, lblCpuLocalLabel, lblCpuLocal);
    cpu_info_handler_.AddLabel(GPS, lblCpuGpsLabel, lblCpuGps);
    cpu_info_handler_.AddLabel(CAM, lblCpuCamLabel, lblCpuCam);

    git_status_handler_.SetLabel(lblCodeSync);
    git_status_handler_.AddLabel(LOCAL, lblCodeSyncLocalLabel, lblCodeSyncLocal);
    git_status_handler_.AddLabel(GPS, lblCodeSyncGpsLabel, lblCodeSyncGps);
    git_status_handler_.AddLabel(CAM, lblCodeSyncCamLabel, lblCodeSyncCam);

    battery_status_handler_.SetLabel(lblBatteryStatus);

    stereo_handler_.SetLabel(lblStereo);

    debug_handler_.SetLabel(lblDebug);

    airspeed_handler_.SetLabel(lblAirspeed);

    UpdateLabels();


    if(!lcm.good()) {
        printf("LCM failed to init... expect issues.");
    }

    lcm.subscribe("log-info-odroid-gps1", &LogSizeHandler::handleMessage, &log_size_handler_);
    lcm.subscribe("log-info-odroid-cam1", &LogSizeHandler::handleMessage, &log_size_handler_);
    lcm.subscribe("log-info-odroid-gps2", &LogSizeHandler::handleMessage, &log_size_handler_);
    lcm.subscribe("log-info-odroid-cam2", &LogSizeHandler::handleMessage, &log_size_handler_);

    lcm.subscribe("cpu-info-odroid-gps1", &CpuInfoHandler::handleMessage, &cpu_info_handler_);
    lcm.subscribe("cpu-info-odroid-cam1", &CpuInfoHandler::handleMessage, &cpu_info_handler_);
    lcm.subscribe("cpu-info-odroid-gps2", &CpuInfoHandler::handleMessage, &cpu_info_handler_);
    lcm.subscribe("cpu-info-odroid-cam2", &CpuInfoHandler::handleMessage, &cpu_info_handler_);

    lcm.subscribe("git-status-odroid-gps1", &GitStatusHandler::handleMessage, &git_status_handler_);
    lcm.subscribe("git-status-odroid-cam1", &GitStatusHandler::handleMessage, &git_status_handler_);
    lcm.subscribe("git-status-odroid-gps2", &GitStatusHandler::handleMessage, &git_status_handler_);
    lcm.subscribe("git-status-odroid-cam2", &GitStatusHandler::handleMessage, &git_status_handler_);

    lcm.subscribe("log-info-AAAZZZA", &LogSizeHandler::handleMessage, &log_size_handler_);
    lcm.subscribe("cpu-info-AAAZZZA", &CpuInfoHandler::handleMessage, &cpu_info_handler_);
    lcm.subscribe("git-status-AAAZZZA", &GitStatusHandler::handleMessage, &git_status_handler_);

    lcm.subscribe("battery-status", &BatteryStatusHandler::handleMessage, &battery_status_handler_);

    lcm.subscribe("debug", &DebugHandler::handleMessage, &debug_handler_);

    lcm.subscribe("airspeed-unchecked", &AirspeedHandler::handleMessage, &airspeed_handler_);

    lcm.subscribe("deltawing_u", &ControllerHandler::handleMessage, &controller_handler_);

    lcm.subscribe("stereo-monitor", &StereoHandler::handleMessage, &stereo_handler_);

    lcm.subscribe("STATE_ESTIMATOR_POSE", &StateEsimatorHandler::handleMessage, &state_estimator_handler_);

    lcm.subscribe("gps", &GpsHandler::handleMessage, &gps_handler_);

}

GoForFlightFrame::~GoForFlightFrame()
{
    //(*Destroy(GoForFlightFrame)
    //*)
}

void GoForFlightFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void GoForFlightFrame::OnAbout(wxCommandEvent& event)
{
    wxString msg = wxbuildinfo(long_f);
    wxMessageBox(msg, _("Welcome to..."));
}

void GoForFlightFrame::OntmrUpdateLcmTrigger(wxTimerEvent& event)
{
    while(NonBlockingLcm(&lcm)) {
        UpdateLabels();
    }

    UpdateLabels();
}

void GoForFlightFrame::UpdateLabels() {

    log_size_handler_.Update();
    state_estimator_handler_.Update();
    gps_handler_.Update();
    cpu_info_handler_.Update();
    battery_status_handler_.Update();
    stereo_handler_.Update();
    controller_handler_.Update();
    debug_handler_.Update();
    airspeed_handler_.Update();
    git_status_handler_.Update();

    chkGoPro->SetForegroundColour(StatusHandler::GetColour(chkGoPro->IsChecked()));
    chkObstacleGoPro->SetForegroundColour(StatusHandler::GetColour(chkObstacleGoPro->IsChecked()));



    if (log_size_handler_.GetStatus()
        && state_estimator_handler_.GetStatus()
        && gps_handler_.GetStatus()
        && cpu_info_handler_.GetStatus()
        && battery_status_handler_.GetStatus()
        && stereo_handler_.GetStatus()
        && controller_handler_.GetStatus()
        && debug_handler_.GetStatus()
        && airspeed_handler_.GetStatus()
        && git_status_handler_.GetStatus()
        && chkGoPro->IsChecked()
        && chkObstacleGoPro->IsChecked()
            ) {

        lblGoForFlight->SetLabel("Go");
        lblGoForFlight->SetForegroundColour(StatusHandler::GetColour(true));

    } else {
        lblGoForFlight->SetLabel("No Go");
        lblGoForFlight->SetForegroundColour(StatusHandler::GetColour(false));
    }

    Layout();

}

/**
 * Processes LCM messages without blocking.
 *
 * @param lcm lcm object
 *
 * @retval true if processed a message
 */
bool GoForFlightFrame::NonBlockingLcm(lcm::LCM *lcm)
{
    // setup an lcm function that won't block when we read it
    int lcm_fd = lcm->getFileno();
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcm_fd, &fds);

    // wait a limited amount of time for an incoming message
    struct timeval timeout = {
        0,  // seconds
        1   // microseconds
    };


    int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

    if(0 == status) {
        // no messages
        //do nothing
        return false;

    } else if(FD_ISSET(lcm_fd, &fds)) {
        // LCM has events ready to be processed.
        lcm->handle();
        return true;
    }
    return false;

}

void GoForFlightFrame::OnchkGoProClick(wxCommandEvent& event)
{
    UpdateLabels();
}

void GoForFlightFrame::OnchkObstacleGoProClick(wxCommandEvent& event)
{
    UpdateLabels();
}

