/* Small wrapper around GTK+
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "gfx.hpp"
#include <gtk/gtk.h>
#include <iostream>
#include <vector>


namespace {
  
  using namespace std;
  
  typedef enum {
    FAKE
  } gdk_flags_t;
}


namespace std {
  
  ostream & operator << (ostream & os, gfx::mouse_flags_t const & flags)
  {
    os << "["
       << (flags & gfx::MOUSE_PRESS ? "Press-" : "")
       << (flags & gfx::MOUSE_RELEASE ? "Release-" : "")
       << (flags & gfx::MOUSE_DRAG ? "Drag-" : "")
       << (flags & gfx::MOUSE_SHIFT ? "Shift-" : "")
       << (flags & gfx::MOUSE_CTRL ? "Ctrl-" : "")
       << (flags & gfx::MOUSE_MOD1 ? "M1-" : "")
       << (flags & gfx::MOUSE_MOD2 ? "M2-" : "")
       << (flags & gfx::MOUSE_MOD3 ? "M3-" : "")
       << (flags & gfx::MOUSE_MOD4 ? "M4-" : "")
       << (flags & gfx::MOUSE_MOD5 ? "M5-" : "")
       << (flags & gfx::MOUSE_B1 ? "B1" : "")
       << (flags & gfx::MOUSE_B2 ? "B2" : "")
       << (flags & gfx::MOUSE_B3 ? "B3" : "")
       << (flags & gfx::MOUSE_B4 ? "B4" : "")
       << (flags & gfx::MOUSE_B5 ? "B5" : "")
       << "]";
    return os;
  }
  
  
  ostream & operator << (ostream & os, gdk_flags_t const & flags)
  {
    os << "["
       << (flags & GDK_SHIFT_MASK ? "Shift-" : "")
       << (flags & GDK_LOCK_MASK ? "Lock-" : "")
       << (flags & GDK_CONTROL_MASK ? "Ctrl-" : "")
       << (flags & GDK_MOD1_MASK ? "M1-" : "")
       << (flags & GDK_MOD2_MASK ? "M2-" : "")
       << (flags & GDK_MOD3_MASK ? "M3-" : "")
       << (flags & GDK_MOD4_MASK ? "M4-" : "")
       << (flags & GDK_MOD5_MASK ? "M5-" : "")
       << (flags & GDK_BUTTON1_MASK ? "B1" : "")
       << (flags & GDK_BUTTON2_MASK ? "B2" : "")
       << (flags & GDK_BUTTON3_MASK ? "B3" : "")
       << (flags & GDK_BUTTON4_MASK ? "B4" : "")
       << (flags & GDK_BUTTON5_MASK ? "B5" : "")
       << "]";
    return os;
  }
  
}


namespace {

  class Button
  {
  public:
    Button (string const & label, void (*callback)())
      : label_(label),
	callback_(callback)
    {}
  
    string const label_;
    void (*callback_)();
  };
  
}


namespace gfx {
    
  static ostream * dbgos = 0;
  static vector<Button*> buttons;
  static GtkWidget * canvas;
  static gint canvas_width, canvas_height;
  static gint canvas_sx, canvas_sy, canvas_x0, canvas_y0;
  static double view_x0, view_y0, view_x1, view_y1;
  static cairo_t * cairo;
    
  static void (*user_idle_cb)();
  static void (*user_draw_cb)();
  static void (*user_mouse_cb)(double, double, int);
    
    
  /* convert view X coordinate to canvas */
  static double v2cx (double vx)
  {
    return canvas_x0 + (vx - view_x0) * canvas_sx;
  }


  /* convert view Y coordinate to canvas */
  static double v2cy (double vy)
  {
    return canvas_y0 + (vy - view_y0) * canvas_sy;
  }


  /* convert lengths form view to canvas */
  static double v2cr (double vr)
  {
    return vr * canvas_sx;
  }
    
    
  static double c2vx (double cx)
  {
    return view_x0 + (cx - canvas_x0) / canvas_sx;
  }
    
    
  static double c2vy (double cy)
  {
    return view_y0 + (cy - canvas_y0) / canvas_sy;
  }
    
    
  static gint cb_idle (gpointer data)
  {
    user_idle_cb ();
    gtk_widget_queue_draw (canvas);
    return TRUE;
  }
    
    
  static gint cb_expose (GtkWidget * ww,
			 GdkEventExpose * ee,
			 gpointer data)
  {
    if (dbgos) {
      *dbgos << __func__ << "\n";
    }
      
    cairo = gdk_cairo_create (ee->window);
  
    cairo_set_source_rgb (cairo, 1.0, 1.0, 1.0);
    cairo_rectangle (cairo, 0, 0, canvas_width, canvas_height);
    cairo_fill (cairo);
      
    /*
      The round line cap style is required for drawing points. It
      may be better to have that switched to and from square
      depending on what we draw, but for now it is easier to just
      globally have round line caps everywhere.
    */
    cairo_set_line_cap (cairo, CAIRO_LINE_CAP_ROUND);
      
    user_draw_cb ();
  
    cairo_destroy (cairo);
    cairo = 0;
  
    return TRUE;
  }


  static void reconf_v2c (gint cw, gint ch,
			  double vx0, double vy0,
			  double vx1, double vy1)
  {
    canvas_width = cw;
    canvas_height = ch;
    view_x0 = vx0;
    view_y0 = vy0;
    view_x1 = vx1;
    view_y1 = vy1;
  
    canvas_sx = (gint) (canvas_width / (view_x1 - view_x0));
    if (canvas_sx < 1) {
      canvas_sx = 1;
    }
    canvas_sy = (gint) (- canvas_height / (view_y1 - view_y0));
    if ( - canvas_sy < 1) {
      canvas_sy = -1;
    }
    if (canvas_sx > - canvas_sy) {
      canvas_sx = - canvas_sy;
    }
    else {
      canvas_sy = - canvas_sx;
    }
    canvas_x0 = (gint) ((canvas_width - (view_x1 - view_x0) * canvas_sx) / 2);
    canvas_y0 = (gint) (canvas_height - (canvas_height + (view_y1 - view_y0) * canvas_sy) / 2);
      
    // if (0 != canvas) {
    // 	gtk_widget_queue_draw (canvas);
    // }
  }


  static gint cb_size_allocate (GtkWidget * ww,
				GtkAllocation * aa,
				gpointer data)
  {
    reconf_v2c (aa->width, aa->height, view_x0, view_y0, view_x1, view_y1);
    return TRUE;
  }


  static void cb_quit (GtkWidget * ww, gpointer data)
  {
    gtk_main_quit();
  }


  static void cb_click (GtkWidget * ww, gpointer data)
  {
    if (dbgos) {
      *dbgos << __func__ << "  " << reinterpret_cast<Button*>(data)->label_ << "\n";
    }
    reinterpret_cast<Button*>(data)->callback_();
    gtk_widget_queue_draw (canvas);
  }
    
    
  int convert_flags (int state, int button)
  {
    int flags (0);
    if (state & GDK_SHIFT_MASK) {
      flags |= MOUSE_SHIFT;
    }
    if (state & GDK_CONTROL_MASK) {
      flags |= MOUSE_CTRL;
    }
    if (state & GDK_MOD1_MASK) {
      flags |= MOUSE_MOD1;
    }
    if (state & GDK_MOD2_MASK) {
      flags |= MOUSE_MOD2;
    }
    if (state & GDK_MOD3_MASK) {
      flags |= MOUSE_MOD3;
    }
    if (state & GDK_MOD4_MASK) {
      flags |= MOUSE_MOD4;
    }
    if (state & GDK_MOD5_MASK) {
      flags |= MOUSE_MOD5;
    }
    if (state & GDK_BUTTON1_MASK || 1 == button) {
      flags |= MOUSE_B1;
    }
    if (state & GDK_BUTTON2_MASK || 2 == button) {
      flags |= MOUSE_B2;
    }
    if (state & GDK_BUTTON3_MASK || 3 == button) {
      flags |= MOUSE_B3;
    }
    if (state & GDK_BUTTON4_MASK || 4 == button) {
      flags |= MOUSE_B4;
    }
    if (state & GDK_BUTTON5_MASK || 5 == button) {
      flags |= MOUSE_B5;
    }
    return flags;
  }
    
    
  static gint cb_mouse_click (GtkWidget * ww,
			      GdkEventButton * bb,
			      gpointer data)
  {
    int flags (convert_flags (bb->state, bb->button));
    if (bb->type == GDK_BUTTON_PRESS) {
      flags |= MOUSE_PRESS;
    }
    else {
      flags |= MOUSE_RELEASE;
    }
      
    if (dbgos) {
      *dbgos << __func__ << "  " << bb->x << " -> " << c2vx (bb->x) << "  " << bb->y
	     << " -> " << c2vy (bb->y) << "  " << (gdk_flags_t) bb->state
	     << " -> " << (mouse_flags_t) flags << "\n";
    }
      
    user_mouse_cb (c2vx (bb->x), c2vy (bb->y), flags);
    gtk_widget_queue_draw (canvas);
      
    return TRUE;
  }
    
    
  static gint cb_mouse_motion (GtkWidget * ww,
			       GdkEventMotion * ee)
  {
    int mx, my;
    GdkModifierType modifier;
    gdk_window_get_pointer(ww->window, &mx, &my, &modifier);

    int flags (convert_flags (ee->state, -1) | MOUSE_DRAG);
      
    if (dbgos) {
      *dbgos << __func__ << "  " << mx << " -> " << c2vx (mx) << "  " << my
	     << " -> " << c2vy (my) << "  " << (gdk_flags_t) ee->state
	     << " -> " << (mouse_flags_t) flags << "\n";
    }
      
    user_mouse_cb (c2vx (mx), c2vy (my), flags);
    gtk_widget_queue_draw (canvas);
      
    return TRUE;
  }
    
    
  void add_button (char const * label, void (*click_callback)())
  {
    buttons.push_back (new Button (label, click_callback));
  }


  void add_button (string const & label, void (*click_callback)())
  {
    buttons.push_back (new Button(label, click_callback));
  }


  void set_view (double x0, double y0, double x1, double y1)
  {
    if (dbgos) {
      *dbgos << __func__ << "  " << x0 << "  " << y0 << "  " << x1 << "  " << y1 << "\n";
    }
    reconf_v2c (canvas_width, canvas_height, x0, y0, x1, y1);
  }


  void set_pen (double width, double red, double green, double blue, double alpha)
  {
    if (dbgos) {
      *dbgos << __func__ << "  " << width
	     << "  " << red << "  " << green << "  " << blue << "  " << alpha << "\n";
    }
    if (!cairo) {
      return;
    }
    cairo_set_source_rgba (cairo, red, green, blue, alpha);
    cairo_set_line_width (cairo, width);
  }
    
    
  void draw_point (double x, double y)
  {
    if (dbgos) {
      *dbgos << __func__ << "  " << x << "  " << y << "\n";
    }
    if (!cairo) {
      return;
    }
    cairo_move_to (cairo, v2cx(x), v2cy(y));
    cairo_line_to (cairo, v2cx(x), v2cy(y));
    cairo_stroke (cairo);
  }
    
    
  void draw_line (double x0, double y0, double x1, double y1)
  {
    if (dbgos) {
      *dbgos << __func__ << "  " << x0 << "  " << y0 << "  " << x1 << "  " << y1
	     << (cairo ? "\n" : "  NO CAIRO\n");
    }
      
    if (!cairo) {
      return;
    }
    cairo_move_to (cairo, v2cx(x0), v2cy(y0));
    cairo_line_to (cairo, v2cx(x1), v2cy(y1));
    cairo_stroke (cairo);
  }


  void draw_arc (double cx, double cy, double rr, double a0, double a1)
  {
    if (!cairo) {
      return;
    }
    cairo_arc (cairo, v2cx(cx), v2cy(cy), v2cr(rr), a0, a1);
    cairo_stroke (cairo);
  }


  void fill_arc (double cx, double cy, double rr, double a0, double a1)
  {
    if (!cairo) {
      return;
    }
    cairo_arc (cairo, v2cx(cx), v2cy(cy), v2cr(rr), a0, a1);
    cairo_fill (cairo);
  }
    
    
  void begin_poly (double x, double y)
  {
    if (!cairo) {
      return;
    }
    cairo_move_to (cairo, v2cx(x), v2cy(y));
  }
    
    
  void add_poly (double x, double y)
  {
    if (!cairo) {
      return;
    }
    cairo_line_to (cairo, v2cx(x), v2cy(y));
  }
    
    
  void draw_poly ()
  {
    if (!cairo) {
      return;
    }
    cairo_stroke (cairo);
  }
    
    
  void fill_poly ()
  {
    if (!cairo) {
      return;
    }
    cairo_fill (cairo);
  }
    
    
  void main (string const & window_title,
	     void (*idle_callback)(),
	     void (*draw_callback)(),
	     void (*mouse_callback)(double px, double py, int flags))
  {
    GtkWidget *window, *vbox, *hbox, *btn;
  
    gtk_init (0, 0);
    view_x0 = -1.0;
    view_y0 = -1.0;
    view_x1 =  1.0;
    view_y1 =  1.0;
    user_idle_cb = idle_callback;
    user_draw_cb = draw_callback;
    user_mouse_cb = mouse_callback;
    cairo = 0;
  
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title ((GtkWindow*) window, window_title.c_str());
  
    vbox = gtk_vbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (window), vbox);
    gtk_widget_show (vbox);
  
    canvas = gtk_drawing_area_new ();
    g_signal_connect (canvas, "expose_event", G_CALLBACK (cb_expose), NULL);
    g_signal_connect (canvas, "size_allocate", G_CALLBACK (cb_size_allocate), NULL);
    g_signal_connect (canvas, "button_press_event", G_CALLBACK (cb_mouse_click), NULL);
    g_signal_connect (canvas, "button_release_event", G_CALLBACK (cb_mouse_click), NULL);
    g_signal_connect (canvas, "motion_notify_event", G_CALLBACK (cb_mouse_motion), NULL);
    gtk_widget_set_events (canvas,
			   GDK_BUTTON_PRESS_MASK |
			   GDK_BUTTON_RELEASE_MASK |
			   GDK_BUTTON_MOTION_MASK);
      
    gtk_widget_show (canvas);
  
    gtk_widget_set_size_request (canvas, 400, 500);
    gtk_box_pack_start (GTK_BOX (vbox), canvas, TRUE, TRUE, 0);
  
    hbox = gtk_hbox_new (TRUE, 3);
    gtk_box_pack_start (GTK_BOX (vbox), hbox, FALSE, TRUE, 0);
    gtk_widget_show (hbox);
  
    btn = gtk_button_new_with_label ("quit");
    g_signal_connect (btn, "clicked", G_CALLBACK (cb_quit), NULL);
    gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
    gtk_widget_show (btn);
  
    for (size_t ii(0); ii < buttons.size(); ++ii) {
      btn = gtk_button_new_with_label (buttons[ii]->label_.c_str());
      g_signal_connect (btn, "clicked", G_CALLBACK (cb_click), buttons[ii]);
      gtk_box_pack_start (GTK_BOX (hbox), btn, TRUE, TRUE, 0);
      gtk_widget_show (btn);
    }
      
    gtk_idle_add (cb_idle, 0);
      
    gtk_widget_show (window);
  
    gtk_main ();
  
    for (size_t ii(0); ii < buttons.size(); ++ii) {
      delete buttons[ii];
    }
    buttons.clear();
  }


  ostream * debug (ostream * debug_os)
  {
    ostream * old = dbgos;
    dbgos = debug_os;
    return old;
  }

}
