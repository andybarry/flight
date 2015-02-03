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
#include <wx/sizer.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/statusbr.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/timer.h>
//*)

#include <lcm/lcm-cpp.hpp>

#include "LogSizeHandler.h"
#include "StateEstimatorHandler.h"

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
        //*)

        //(*Identifiers(GoForFlightFrame)
        static const long ID_STATICTEXT2;
        static const long ID_STATICTEXT5;
        static const long ID_STATICTEXT3;
        static const long ID_STATICTEXT4;
        static const long ID_STATICTEXT1;
        static const long ID_PANEL1;
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        static const long ID_TIMER1;
        //*)

        //(*Declarations(GoForFlightFrame)
        wxStaticText* lblStateEsimator;
        wxStaticText* lblTimesync;
        wxPanel* Panel1;
        wxStatusBar* StatusBar1;
        wxStaticText* lblTimestamp;
        wxStaticText* lblLogging;
        wxTimer tmrUpdateLcm;
        wxStaticText* lblGoForFlight;
        //*)

        void UpdateLabels();
        bool NonBlockingLcm(lcm::LCM *lcm);

        lcm::LCM lcm;


        LogSizeHandler log_size_handler_;
        StateEsimatorHandler state_estimator_handler_;

        DECLARE_EVENT_TABLE()
};

#endif // GOFORFLIGHTMAIN_H
