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
const long GoForFlightFrame::ID_STATICTEXT5 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT3 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT4 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT6 = wxNewId();
const long GoForFlightFrame::ID_STATICTEXT1 = wxNewId();
const long GoForFlightFrame::ID_PANEL1 = wxNewId();
const long GoForFlightFrame::idMenuQuit = wxNewId();
const long GoForFlightFrame::idMenuAbout = wxNewId();
const long GoForFlightFrame::ID_STATUSBAR1 = wxNewId();
const long GoForFlightFrame::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(GoForFlightFrame,wxFrame)
    //(*EventTable(GoForFlightFrame)
    //*)
END_EVENT_TABLE()

GoForFlightFrame::GoForFlightFrame(wxWindow* parent,wxWindowID id)
{

    if(!lcm.good()) {
        printf("LCM failed to init... expect issues.");
    }

    lcm.subscribe("log_size", &LogSizeHandler::handleMessage, &log_size_handler_);
    lcm.subscribe("STATE_ESTIMATOR_POSE", &StateEsimatorHandler::handleMessage, &state_estimator_handler_);
    lcm.subscribe("gps", &GpsHandler::handleMessage, &gps_handler_);


    //(*Initialize(GoForFlightFrame)
    wxMenuItem* MenuItem2;
    wxMenuItem* MenuItem1;
    wxBoxSizer* BoxSizer3;
    wxMenu* Menu1;
    wxBoxSizer* BoxSizer2;
    wxBoxSizer* BoxSizer4;
    wxBoxSizer* BoxSizer1;
    wxMenuBar* MenuBar1;
    wxMenu* Menu2;

    Create(parent, wxID_ANY, _("Go For Flight"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    Panel1 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    BoxSizer3 = new wxBoxSizer(wxVERTICAL);
    lblTimesync = new wxStaticText(Panel1, ID_STATICTEXT2, _("Timesync: Offline"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    BoxSizer3->Add(lblTimesync, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer4 = new wxBoxSizer(wxHORIZONTAL);
    BoxSizer4->Add(46,20,0, wxALL|wxFIXED_MINSIZE|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    lblTimestamp = new wxStaticText(Panel1, ID_STATICTEXT5, _("--:--:--"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    BoxSizer4->Add(lblTimestamp, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer3->Add(BoxSizer4, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 0);
    lblLogging = new wxStaticText(Panel1, ID_STATICTEXT3, _("Logging: Offline (#--)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    BoxSizer3->Add(lblLogging, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblStateEsimator = new wxStaticText(Panel1, ID_STATICTEXT4, _("State Esimator: Offline"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    BoxSizer3->Add(lblStateEsimator, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    lblGps = new wxStaticText(Panel1, ID_STATICTEXT6, _("GPS: Offline (Sats: --)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    BoxSizer3->Add(lblGps, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer2->Add(BoxSizer3, 4, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer2->Add(49,20,1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    lblGoForFlight = new wxStaticText(Panel1, ID_STATICTEXT1, _("No Go"), wxDefaultPosition, wxDefaultSize, wxST_NO_AUTORESIZE|wxALIGN_CENTRE, _T("ID_STATICTEXT1"));
    BoxSizer2->Add(lblGoForFlight, 0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer2->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
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
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -1 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    tmrUpdateLcm.SetOwner(this, ID_TIMER1);
    tmrUpdateLcm.Start(100, false);
    BoxSizer1->Fit(this);
    BoxSizer1->SetSizeHints(this);

    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&GoForFlightFrame::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&GoForFlightFrame::OnAbout);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&GoForFlightFrame::OntmrUpdateLcmTrigger);
    //*)


    log_size_handler_.SetLoggingLabel(lblLogging);
    log_size_handler_.SetTimesyncLabels(lblTimesync, lblTimestamp);

    state_estimator_handler_.SetLabel(lblStateEsimator);
    gps_handler_.SetLabel(lblGps);

    UpdateLabels();


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
    NonBlockingLcm(&lcm);

    UpdateLabels();
}

void GoForFlightFrame::UpdateLabels() {

    log_size_handler_.UpdateLabel();


    state_estimator_handler_.UpdateLabel();
    gps_handler_.UpdateLabel();



    if (log_size_handler_.GetStatus()
        && state_estimator_handler_.GetStatus()
        && gps_handler_.GetStatus()
            ) {

        lblGoForFlight->SetLabel("Go");
        lblGoForFlight->SetForegroundColour(StatusHandler::GetColour(true));
    } else {
        lblGoForFlight->SetLabel("No Go");
        lblGoForFlight->SetForegroundColour(StatusHandler::GetColour(false));
    }

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
