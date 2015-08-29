/***************************************************************
 * Name:      GoForFlightMain.h
 * Purpose:   Defines Application Frame
 * Author:    Andy Barry (abarry@gmail.com)
 * Created:   2015-02-03
 * Copyright: Andy Barry (abarry.org)
 * License:
 **************************************************************/

#ifndef GOFORFLIGHTMAIN_H
#define GOFORFLIGHTMAIN_H

//(*Headers(GoForFlightFrame)
#include <wx/checkbox.h>
#include <wx/sizer.h>
#include <wx/menu.h>
#include <wx/radiobut.h>
#include <wx/panel.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/timer.h>
//*)

#include <lcm/lcm-cpp.hpp>

#include "LogSizeHandler.h"
#include "StateEstimatorHandler.h"
#include "GpsHandler.h"
#include "CpuInfoHandler.h"
#include "BatteryStatusHandler.h"
#include "StereoHandler.h"
#include "ControllerHandler.h"
#include "DebugHandler.h"
#include "AirspeedHandler.h"
#include "GitStatusHandler.h"

class GoForFlightFrame: public wxFrame
{
    public:

        GoForFlightFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~GoForFlightFrame();

    private:

        //(*Handlers(GoForFlightFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnTimer1Trigger(wxTimerEvent& event);
        void OntmrUpdateLcmTrigger(wxTimerEvent& event);
        void OnClose(wxCloseEvent& event);
        void OnchkGoProClick(wxCommandEvent& event);
        void OnchkObstacleGoProClick(wxCommandEvent& event);
        //*)

        //(*Identifiers(GoForFlightFrame)
        static const long ID_STATICTEXT2;
        static const long ID_STATICTEXT30;
        static const long ID_STATICTEXT31;
        static const long ID_STATICTEXT20;
        static const long ID_STATICTEXT5;
        static const long ID_STATICTEXT21;
        static const long ID_STATICTEXT22;
        static const long ID_STATICTEXT3;
        static const long ID_STATICTEXT14;
        static const long ID_STATICTEXT7;
        static const long ID_STATICTEXT15;
        static const long ID_STATICTEXT8;
        static const long ID_STATICTEXT16;
        static const long ID_STATICTEXT9;
        static const long ID_STATICTEXT36;
        static const long ID_STATICTEXT6;
        static const long ID_STATICTEXT4;
        static const long ID_STATICTEXT34;
        static const long ID_RADIOBUTTON1;
        static const long ID_RADIOBUTTON2;
        static const long ID_STATICTEXT32;
        static const long ID_STATICTEXT33;
        static const long ID_CHECKBOX1;
        static const long ID_CHECKBOX2;
        static const long ID_STATICTEXT1;
        static const long ID_STATICTEXT10;
        static const long ID_STATICTEXT17;
        static const long ID_STATICTEXT11;
        static const long ID_STATICTEXT18;
        static const long ID_STATICTEXT12;
        static const long ID_STATICTEXT19;
        static const long ID_STATICTEXT13;
        static const long ID_STATICTEXT23;
        static const long ID_STATICTEXT24;
        static const long ID_STATICTEXT25;
        static const long ID_STATICTEXT26;
        static const long ID_STATICTEXT27;
        static const long ID_STATICTEXT28;
        static const long ID_STATICTEXT29;
        static const long ID_STATICTEXT38;
        static const long ID_STATICTEXT37;
        static const long ID_STATICTEXT39;
        static const long ID_STATICTEXT40;
        static const long ID_STATICTEXT41;
        static const long ID_STATICTEXT42;
        static const long ID_STATICTEXT43;
        static const long ID_STATICTEXT35;
        static const long ID_PANEL1;
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_TIMER1;
        //*)

        //(*Declarations(GoForFlightFrame)
        wxStaticText* lblCodeSyncCam;
        wxStaticText* lblStateEsimator;
        wxStaticText* lblDiskFreeLocal;
        wxCheckBox* chkObstacleGoPro;
        wxStaticText* lblLogCam;
        wxStaticText* lblTimesync;
        wxStaticText* lblAirspeed;
        wxStaticText* lblCodeSync;
        wxStaticText* lblCpuCamLabel;
        wxPanel* Panel1;
        wxRadioButton* rdoThrow;
        wxStaticText* lblDebug;
        wxStaticText* lblCodeSyncLocal;
        wxStaticText* lblTimesyncGps;
        wxStaticText* lblDiskFreeLabelCam;
        wxStaticText* lblCodeSyncGps;
        wxStaticText* lblTimesyncLabelCam;
        wxStaticText* lblCpuCam;
        wxStaticText* lblTimesyncLocal;
        wxStaticText* lblCpuGpsLabel;
        wxStaticText* lblController;
        wxStaticText* lblCodeSyncCamLabel;
        wxStaticText* lblDiskFreeCam;
        wxCheckBox* chkGoPro;
        wxRadioButton* rdoLauncher;
        wxStaticText* lblCodeSyncGpsLabel;
        wxStaticText* lblTimesyncCam;
        wxStaticText* lblDiskFreeGps;
        wxStaticText* lblLogGpsLabel;
        wxStaticText* lblCpuLocal;
        wxStaticText* lblCodeSyncLocalLabel;
        wxStaticText* lblTimesyncLabelLocal;
        wxStaticText* lblGps;
        wxStaticText* lblTimesyncLabelGps;
        wxStaticText* lblDiskFreeLabelGps;
        wxStaticText* lblLogGps;
        wxStaticText* lblLogging;
        wxStaticText* lblBatteryStatus;
        wxStaticText* lblLogCamLabel;
        wxStaticText* lblCpuGps;
        wxStaticText* lblDiskFree;
        wxTimer tmrUpdateLcm;
        wxStaticText* lblDiskFreeLabelLocal;
        wxStaticText* lblCpu;
        wxStaticText* lblCpuLocalLabel;
        wxStaticText* lblLogLocalLabel;
        wxStaticText* lblGoForFlight;
        wxStaticText* lblLogLocal;
        wxStaticText* lblStereo;
        //*)

        void UpdateLabels();
        bool NonBlockingLcm(lcm::LCM *lcm);

        lcm::LCM lcm;


        LogSizeHandler log_size_handler_;
        StateEsimatorHandler state_estimator_handler_;
        GpsHandler gps_handler_;
        CpuInfoHandler cpu_info_handler_;
        BatteryStatusHandler battery_status_handler_;
        StereoHandler stereo_handler_;
        ControllerHandler controller_handler_;
        DebugHandler debug_handler_;
        AirspeedHandler airspeed_handler_;
        GitStatusHandler git_status_handler_;


        DECLARE_EVENT_TABLE()
};

#endif // GOFORFLIGHTMAIN_H
