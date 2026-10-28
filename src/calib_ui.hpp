#pragma once

// Custom Pangolin views for the unified calibration tool.
//
//   ExitButtonView - always-on-top button pinned to the top-right corner of
//                    the window; clicking quits the whole node cleanly (no
//                    terminal Ctrl-C needed). Drawn with plain geometry
//                    only (no font rendering).
//
// All textual UI lives in native Pangolin panels instead (their widgets
// render reliably); TopicRow is the topic description shared with the node.

#include <pangolin/pangolin.h>

#include <string>

namespace msc {

struct TopicRow {
  std::string name;
  std::string datatype; // PointCloud2 / Image / MarkerArray
  bool is_image = false;
  bool is_marker = false;
};

class ExitButtonView;

class ExitButtonHandler : public pangolin::Handler {
public:
  void Mouse(pangolin::View &v, pangolin::MouseButton button, int x, int y,
             bool pressed, int button_state) override;
  void PassiveMouseMotion(pangolin::View &v, int x, int y,
                          int button_state) override;
  void MouseMotion(pangolin::View &v, int x, int y, int button_state) override;

private:
  friend class ExitButtonView;
  int hover_x_ = -1, hover_y_ = -1;
};

class ExitButtonView : public pangolin::View {
  friend class ExitButtonHandler;

public:
  ExitButtonView() {
    handler_ = new ExitButtonHandler();
    SetHandler(handler_);
  }
  ~ExitButtonView() { delete handler_; }

  void Render() override {
    const pangolin::Viewport vp = GetBounds();
    vp.Activate(); // v0.6 RenderChildren() does NOT activate child viewports
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, vp.w, 0, vp.h, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    if (vp.Contains(handler_->hover_x_, handler_->hover_y_))
      glColor3f(0.85f, 0.15f, 0.12f);
    else
      glColor3f(0.55f, 0.13f, 0.12f);
    glBegin(GL_QUADS);
    glVertex2f(0, 0);
    glVertex2f(vp.w, 0);
    glVertex2f(vp.w, vp.h);
    glVertex2f(0, vp.h);
    glEnd();

    // white cross, pure geometry (no font involved)
    const float m = 0.28f;
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glVertex2f(vp.w * m, vp.h * m);
    glVertex2f(vp.w * (1.0f - m), vp.h * (1.0f - m));
    glVertex2f(vp.w * (1.0f - m), vp.h * m);
    glVertex2f(vp.w * m, vp.h * (1.0f - m));
    glEnd();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  }

private:
  ExitButtonHandler *handler_ = nullptr;
};

inline void ExitButtonHandler::Mouse(pangolin::View &v,
                                     pangolin::MouseButton button, int x,
                                     int y, bool pressed, int button_state) {
  if (pressed) {
    std::cout << "\n[exit] shutdown requested from GUI" << std::endl;
    pangolin::Quit();
  }
}
inline void ExitButtonHandler::PassiveMouseMotion(pangolin::View &v, int x,
                                                  int y, int button_state) {
  hover_x_ = x;
  hover_y_ = y;
}
inline void ExitButtonHandler::MouseMotion(pangolin::View &v, int x, int y,
                                           int button_state) {
  hover_x_ = x;
  hover_y_ = y;
}

} // namespace msc
