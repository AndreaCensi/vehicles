#!/usr/bin/env python
# -*- coding: utf-8 -*-
# generated by wxGlade HG on Sat Jul  9 21:41:09 2011

import wx

# begin wxGlade: extracode
# end wxGlade

class MainFrame(wx.MDIChildFrame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: MainFrame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.MDIChildFrame.__init__(self, *args, **kwds)
        self.panel_1 = wx.Panel(self, -1, style=wx.SIMPLE_BORDER)
        self.bg_type = wx.RadioBox(self.panel_1, -1, "Type", choices=["Vehicles simulation", "Landroids", "Logged data"], majorDimension=0, style=wx.RA_SPECIFY_ROWS)
        self.bg_logging = wx.Choice(self.panel_1, -1, choices=["log nothing", "log essential data  (not implemented)", "log everything"])
        self.bg_visualization = wx.Choice(self.panel_1, -1, choices=["No visualization", "Minimal (world geometry and robot)", "Medium (+sensels as image)", "Fancy (sensor data)"])
        self.bg_publish = wx.Choice(self.panel_1, -1, choices=["Do not publish agent information", "Publish rarely", "Publish often"])
        self.bg_button_run = wx.Button(self.panel_1, -1, "Create .launch for ROS execution")
        self.bg_button_py = wx.Button(self.panel_1, -1, "Create .py for local execution")

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_BUTTON, self.h_button_run, self.bg_button_run)
        # end wxGlade

    def __set_properties(self):
        # begin wxGlade: MainFrame.__set_properties
        self.SetTitle("Bootstrapping GUI")
        self.bg_type.Enable(False)
        self.bg_type.SetSelection(0)
        self.bg_logging.SetSelection(2)
        self.bg_visualization.SetSelection(3)
        self.bg_publish.SetSelection(1)
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: MainFrame.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_2 = wx.BoxSizer(wx.VERTICAL)
        sizer_2.Add(self.bg_type, 0, 0, 0)
        sizer_2.Add(self.bg_logging, 0, 0, 0)
        sizer_2.Add(self.bg_visualization, 0, 0, 0)
        sizer_2.Add(self.bg_publish, 0, 0, 0)
        sizer_2.Add(self.bg_button_run, 0, 0, 0)
        sizer_2.Add(self.bg_button_py, 0, 0, 0)
        self.panel_1.SetSizer(sizer_2)
        sizer_1.Add(self.panel_1, 2, wx.EXPAND, 0)
        self.SetSizer(sizer_1)
        sizer_1.Fit(self)
        self.Layout()
        # end wxGlade

    def h_button_run(self, event): # wxGlade: MainFrame.<event_handler>
        print "Event handler `h_button_run' not implemented!"
        event.Skip()

# end of class MainFrame


class ConfigFrame(wx.MDIChildFrame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: ConfigFrame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.MDIChildFrame.__init__(self, *args, **kwds)
        self.panel_2 = wx.Panel(self, -1)
        self.bg_choice = wx.Choice(self.panel_2, -1, choices=[])
        self.bg_save = wx.Button(self.panel_2, wx.ID_SAVE, "")
        self.bg_desc = wx.StaticText(self.panel_2, -1, "Description\n\n\n\n\n\n")
        self.sizer_4_staticbox = wx.StaticBox(self.panel_2, -1, "")
        self.panel_3 = wx.Panel(self, -1)
        self.bg_config = wx.TextCtrl(self.panel_3, -1, "This is \na  multiline configuration\n\nek\n", style=wx.TE_PROCESS_ENTER|wx.TE_MULTILINE|wx.TE_LINEWRAP|wx.TE_WORDWRAP|wx.NO_BORDER)
        self.bg_config_status = wx.StaticText(self.panel_3, -1, "This is the minimum size for this widget more more\n\none two")

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_CHOICE, self.h_choice_changed, self.bg_choice)
        self.Bind(wx.EVT_BUTTON, self.h_save_pressed, self.bg_save)
        self.Bind(wx.EVT_TEXT, self.h_text, self.bg_config)
        # end wxGlade

    def __set_properties(self):
        # begin wxGlade: ConfigFrame.__set_properties
        self.SetTitle("frame_2")
        self.bg_save.Enable(False)
        self.bg_config.SetForegroundColour(wx.Colour(0, 0, 1))
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: ConfigFrame.__do_layout
        sizer_3 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_7 = wx.GridSizer(1, 1, 10, 10)
        self.sizer_4_staticbox.Lower()
        sizer_4 = wx.StaticBoxSizer(self.sizer_4_staticbox, wx.VERTICAL)
        sizer_6 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_4.Add(self.bg_choice, 0, wx.EXPAND, 0)
        sizer_6.Add(self.bg_save, 0, 0, 0)
        sizer_6.Add((20, 20), 0, 0, 0)
        sizer_4.Add(sizer_6, 1, wx.EXPAND, 0)
        sizer_4.Add(self.bg_desc, 0, wx.EXPAND, 0)
        self.panel_2.SetSizer(sizer_4)
        sizer_3.Add(self.panel_2, 1, wx.EXPAND, 0)
        sizer_7.Add(self.bg_config, 0, wx.EXPAND, 0)
        sizer_7.Add(self.bg_config_status, 0, wx.EXPAND|wx.SHAPED, 0)
        self.panel_3.SetSizer(sizer_7)
        sizer_3.Add(self.panel_3, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_3)
        sizer_3.Fit(self)
        sizer_3.SetSizeHints(self)
        self.Layout()
        # end wxGlade

    def h_choice_changed(self, event): # wxGlade: ConfigFrame.<event_handler>
        print "Event handler `h_choice_changed' not implemented!"
        event.Skip()

    def h_save_pressed(self, event): # wxGlade: ConfigFrame.<event_handler>
        print "Event handler `h_save_pressed' not implemented!"
        event.Skip()

    def h_text_enter(self, event): # wxGlade: ConfigFrame.<event_handler>
        print "Event handler `h_text_enter' not implemented!"
        event.Skip()

    def h_text(self, event): # wxGlade: ConfigFrame.<event_handler>
        print "Event handler `h_text' not implemented!"
        event.Skip()

# end of class ConfigFrame


if __name__ == "__main__":
    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    frame_1 = (None, -1, "")
    app.SetTopWindow(frame_1)
    frame_1.Show()
    app.MainLoop()
