#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/utils/motion.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define TIME_STEP 64

// ── Detection ─────────────────────────────────
#define SCAN_ROWS          80   
#define CLUSTER_MIN_WIDTH  3
#define WHITE_R_MIN        100  // Lowered for shadow handling
#define WHITE_G_MIN        100
#define WHITE_B_MIN        100

// ── Kick Conditions ───────────────────────────
#define TOP_KICK_THRESHOLD 0.82 // Based on your logs (kicks if Y > ~98/120)
#define BOT_KICK_THRESHOLD 0.30 // Kicks if visible in bottom cam at all
#define CENTER_TOLERANCE   75   

static WbDeviceTag cam_top, cam_bot;
static int width_top, height_top;
static int width_bot, height_bot;

static WbMotionRef motion_left, motion_right, motion_forward, motion_kick;

typedef enum { STATE_ACTIVE, STATE_COOLDOWN } RobotState;
static RobotState state = STATE_ACTIVE;
static int cooldown_timer = 0;
static int last_top_y = 0;

// ─────────────────────────────────────────────
bool is_white(int r, int g, int b) {
  return (r > WHITE_R_MIN && g > WHITE_G_MIN && b > WHITE_B_MIN &&
          abs(r - g) < 45 && abs(r - b) < 45);
}

int detect_ball(const unsigned char *image, int width, int height, int *out_width, int *out_y) {
  int best_size = 0, best_x = -1, best_y = -1;
  if (!image) return -1;

  for (int row = 0; row < SCAN_ROWS; row++) {
    int y = height - 1 - row;
    if (y < 0) break;
    int cluster_start = -1, cluster_w = 0;
    for (int x = 0; x < width; x++) {
      if (is_white(wb_camera_image_get_red(image, width, x, y),
                   wb_camera_image_get_green(image, width, x, y),
                   wb_camera_image_get_blue(image, width, x, y))) {
        if (cluster_start == -1) cluster_start = x;
        cluster_w++;
      } else if (cluster_start != -1) {
        if (cluster_w >= CLUSTER_MIN_WIDTH && cluster_w > best_size) {
          best_size = cluster_w; best_x = cluster_start + cluster_w / 2; best_y = y;
        }
        cluster_start = -1; cluster_w = 0;
      }
    }
  }
  if (out_width) *out_width = best_size;
  if (out_y) *out_y = best_y;
  return best_x;
}

void play(WbMotionRef motion, const char *label) {
  printf("▶ ACTION: %s\n", label);
  wbu_motion_play(motion);
  while (!wbu_motion_is_over(motion)) wb_robot_step(TIME_STEP);
}

int main() {
  wb_robot_init();
  
  cam_top = wb_robot_get_device("CameraTop");
  cam_bot = wb_robot_get_device("CameraBottom");
  wb_camera_enable(cam_top, TIME_STEP);
  wb_camera_enable(cam_bot, TIME_STEP);
  
  width_top = wb_camera_get_width(cam_top);
  height_top = wb_camera_get_height(cam_top);
  width_bot = wb_camera_get_width(cam_bot);
  height_bot = wb_camera_get_height(cam_bot);

  motion_left    = wbu_motion_new("../../motions/SideStepLeft.motion");
  motion_right   = wbu_motion_new("../../motions/SideStepRight.motion");
  motion_forward = wbu_motion_new("../../motions/Forwards50.motion");
  motion_kick    = wbu_motion_new("../../motions/Shoot.motion");

  printf("🤖 NAO READY. Watching for ball...\n");

  while (wb_robot_step(TIME_STEP) != -1) {
    if (state == STATE_COOLDOWN) {
      if (--cooldown_timer <= 0) state = STATE_ACTIVE;
      continue;
    }

    int top_w, top_y, bot_w, bot_y;
    int top_x = detect_ball(wb_camera_get_image(cam_top), width_top, height_top, &top_w, &top_y);
    int bot_x = detect_ball(wb_camera_get_image(cam_bot), width_bot, height_bot, &bot_w, &bot_y);

    printf("LOG: TOP(x=%d y=%d) BOT(x=%d y=%d) | LastY=%d\n", top_x, top_y, bot_x, bot_y, last_top_y);

    // --- PRIORITY 1: KICKING (Distance is more important than Alignment) ---
    bool is_at_feet_top = (top_x != -1 && top_y > (height_top * TOP_KICK_THRESHOLD));
    bool is_at_feet_bot = (bot_x != -1);
    bool ball_vanished_under_chin = (top_x == -1 && bot_x == -1 && last_top_y > (height_top * 0.80));

    if (is_at_feet_top || is_at_feet_bot || ball_vanished_under_chin) {
      play(motion_kick, "KICKING BALL");
      state = STATE_COOLDOWN;
      cooldown_timer = 25; // wait for physics to settle
      last_top_y = 0;
      continue;
    }

    // --- PRIORITY 2: SEARCHING ---
    if (top_x == -1) {
      play(motion_left, "SEARCHING (Left Scan)");
      continue;
    }

    // --- PRIORITY 3: NAVIGATION ---
    last_top_y = top_y; // Save current position for memory-kick check
    int center = width_top / 2;

    if (top_x < center - CENTER_TOLERANCE) {
      play(motion_left, "ALIGNING LEFT");
    } else if (top_x > center + CENTER_TOLERANCE) {
      play(motion_right, "ALIGNING RIGHT");
    } else {
      play(motion_forward, "WALKING TO BALL");
    }
  }

  wb_robot_cleanup();
  return 0;
}
