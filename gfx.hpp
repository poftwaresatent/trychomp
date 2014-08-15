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

/**
   \file gfx.hpp
   
   Simplified GUI wrapper around GTK, GDK, and Cairo.  By using this
   wrapper, you can easily create a custom graphical output with a few
   buttons that will be displayed along the bottom of the GUI window.
   Processing mouse events like clicking and dragging is also kept
   very simple.
   
   The basic approach is that you register various callback functions
   via function pointers.  Thus, for each custom button, you specify a
   separate function to gfx::add_button.  This function will
   get called when the corresponding button is clicked.  Similarly,
   your custom mouse callback is specified as a function pointer to
   the gfx::main.  Your custom drawing function gets
   registered there as well.  Various functions are provided so that
   you can draw shapes on the GUI.
   
   The gfx::main function gets called as the last action in
   your program.  it sets up GTK for you and enters its event handling
   loop.
*/

#ifndef GFX_HPP
#define GFX_HPP

#include <string>


namespace gfx {

  using namespace std;
    
  
  /**
     Flags that contain information about mouse events.  Unifies
     data from various GDK and GTK event data structures.
  */
  typedef enum {
    MOUSE_PRESS   = 1 <<  1, /**< the mouse has been pressed */
    MOUSE_RELEASE = 1 <<  2, /**< the mouse has been release */
    MOUSE_DRAG    = 1 <<  3, /**< the mouse has been dragged */
    MOUSE_B1      = 1 <<  4, /**< button 1 is involved (on Linux: left) */
    MOUSE_B2      = 1 <<  5, /**< button 2 is involved (on Linux: middle) */
    MOUSE_B3      = 1 <<  6, /**< button 3 is involved (on Linux: right) */
    MOUSE_B4      = 1 <<  7, /**< button 4 is involved */
    MOUSE_B5      = 1 <<  8, /**< button 5 is involved */
    MOUSE_SHIFT   = 1 <<  9, /**< Shift was held down when the mouse was pressed */
    MOUSE_CTRL    = 1 << 10, /**< Control was held down when the mouse was pressed */
    MOUSE_MOD1    = 1 << 11, /**< Modifier key 1 (usually Alt) was held down */
    MOUSE_MOD2    = 1 << 12, /**< Modifier key 2 was held down */
    MOUSE_MOD3    = 1 << 13, /**< Modifier key 3 was held down */
    MOUSE_MOD4    = 1 << 14, /**< Modifier key 4 was held down */
    MOUSE_MOD5    = 1 << 15  /**< Modifier key 5 was held down */
  } mouse_flags_t;
    
    
  /**
     Specify a custom button.  It will be labelled with the provided
     string, and the provided function will be called each time the
     button is clicked.
      
     \note This function has to be called /before/ you call
     gfx::main.
  */
  void add_button (string const & label, void (*click_callback)());
    
    
  /**
     Specify which region of the planar coordinate system should be
     drawn inside the GUI.  This sets up the transformation between
     pixel coordinates (which change when the GUI gets resized, for
     example) and the coordinates that you care about.
       
     \note The idea is for you to call set_view as one of the first
     things in your draw callback.  When called from /outside/ your
     draw callback, the GUI might not refresh automatically.
  */
  void set_view (double x0, double y0, double x1, double y1);
    
    
  /**
     Specify the widh and the color of the pen with which to draw
     things.  It affects all subsequent drawing functions.  The width
     is specified in number of pixels.
      
     \note Only works when called from within a draw callback.
  */
  void set_pen (double width, double red, double green, double blue, double alpha);
    
    
  /**
     Draw a point with the current pen settings.  The size of the
     point is determined by the 'width' parameter of set_pen.
      
     \note Only works when called from within a draw callback.
  */
  void draw_point (double x, double y);
    
    
  /**
     Draw a straight line segment from (x0, y0) to (x1, y1) using
     the current pen settings.
       
     \note Only works when called from within a draw callback.
  */
  void draw_line (double x0, double y0, double x1, double y1);
    
    
  /**
     Draw an arc of circle with the current pen settings.  The
     circle is centered at (cx, cy) and has radius rr.  The arc
     starts at the angle a0 and is drawn to angle a1.  Angles are
     specified in radians, and a0 must be smaller than a1 to avoid
     wrapping effects.
       
     \note Only works when called from within a draw callback.
  */
  void draw_arc (double cx, double cy, double rr, double a0, double a1);
    
    
  /**
     Similar to gfx::draw_arc, but instead of an outline it
     fills the specified circular arc using the current pen
     settings.
       
     \note Only works when called from within a draw callback.
  */
  void fill_arc (double cx, double cy, double rr, double a0, double a1);
    
    
  /**
     Start drawing a polygon at the given point, using current pen
     settings. Call add_poly for each subsequent point, and when you
     are done call draw_poly or fill_poly.
       
     \note Only works when called from within a draw callback.
     The pen settings should be given before begin_poly is called.
  */
  void begin_poly (double x, double y);
    
    
  /**
     Continue drawing a polygon that was started with
     gfx::begin_poly.
       
     \note Only works when called from within a draw callback.
  */
  void add_poly (double x, double y);
    
    
  /**
     Finish drawing a polygon in outline mode.  Call this function
     after an appropriate sequence of begin_poly and add_poly calls.
       
     \note Only works when called from within a draw callback.
  */
  void draw_poly ();
    
    
  /**
     Finish drawing a polygon in filled mode.  Call this function
     after an appropriate sequence of begin_poly and add_poly calls.
       
     \note Only works when called from within a draw callback.
  */
  void fill_poly ();
    
    
  /**
     Set up the GUI and enter its event processing loop.  You have
     to specify valid function pointers for the idle_callback, the
     draw_callback, and the mouse_callback.  Any extra buttons need
     to be set up before by calling gfx::add_button.
       
     The given draw_callback function called each time the scene
     should be drawn. Inside the draw_callback, use the various
     other functions to actually draw things.
       
     Similarly, the given mouse_callback is called each time a mouse
     button is pressed, released, or dragged.  The coordinates where
     this happened is passed as (px, py) to your mouse callback.
     The transformation from pixel coordinates to your view
     coordinates are done for you, based on the latest settings that
     you gave to the gfx::set_view function.  The flags
     passed to the mouse callback are a bitwise-or of the
     gfx::mouse_flags_t type.  You can thus find out which
     button was pressed or dragged or so, and also whether any
     modifier keys (such as Shift of Control) were pressed.
       
     \note This function only returns when the user has requested to
     quit the application.
  */
  void main (string const & window_title,
	     void (*idle_callback)(),
	     void (*draw_callback)(),
	     void (*mouse_callback)(double px, double py, int flags));
    
    
  /**
     Control debug messages of this GUI wrapper.  By passing a
     pointer to a valid ostream, you can capture those
     messages.  If the given debug_os pointer is null (the default
     setting), no debug messages will be generated.
       
     \return The old debug_os pointer, in case you need to restore
     it later.
  */
  ostream * debug (ostream * debug_os);
    
}

#endif // GFX_HPP
