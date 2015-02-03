/***************************************************************
 * Name:      GoForFlightApp.cpp
 * Purpose:   Code for Application Class
 * Author:    Andy Barry (abarry@gmail.com)
 * Created:   2015-02-03
 * Copyright: Andy Barry (abarry.org)
 * License:
 **************************************************************/

#include "GoForFlightApp.h"

//(*AppHeaders
#include "GoForFlightMain.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(GoForFlightApp);

bool GoForFlightApp::OnInit()
{
    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
    	GoForFlightFrame* Frame = new GoForFlightFrame(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}
