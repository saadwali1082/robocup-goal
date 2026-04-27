#include <webots/robot.h>
#include <webots/utils/motion.h>
#include <webots/camera.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#define TIME_STEP 32
#define KICK_THRESHOLD 48
#define CENTER_TARGET 160  

typedef struct {
    int detected;
    int size;
    int centerX;
    int centerY;
    int leftEdge;
    int rightEdge;
    int redPixels;
} BallInfo;

// Structure to save kick data
typedef struct {
    int timestamp;
    int ballSize;
    int ballCenterX;
    int ballCenterY;
    int ballLeftEdge;
    int ballRightEdge;
    int redPixelCount;
    int walkStepsTaken;
    int sidestepDirection; 
    char kickType[50];
} KickData;

// Global variable to store kick data
KickData lastKickData;

// Ball detection
BallInfo detectBallFromCamera(WbDeviceTag camera) {
    BallInfo info;
    info.detected = 0;
    info.size = 0;
    info.centerX = -1;
    info.centerY = -1;
    info.leftEdge = -1;
    info.rightEdge = -1;
    info.redPixels = 0;
    
    const unsigned char *image = wb_camera_get_image(camera);
    if (!image) return info;
    
    int width = wb_camera_get_width(camera);
    int height = wb_camera_get_height(camera);
    
    int minX = width, maxX = 0;
    int minY = height, maxY = 0;
    
    for (int y = 0; y < height; y += 2) {
        for (int x = 0; x < width; x += 2) {
            int r = wb_camera_image_get_red(image, width, x, y);
            int g = wb_camera_image_get_green(image, width, x, y);
            int b = wb_camera_image_get_blue(image, width, x, y);
            
            if (r > 80 && r > g && r > b) {
                info.redPixels++;
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
        }
    }
    
    if (info.redPixels > 15) {
        info.detected = 1;
        info.leftEdge = minX;
        info.rightEdge = maxX;
        info.size = (maxX - minX + maxY - minY) / 2;
        info.centerX = (minX + maxX) / 2;
        info.centerY = (minY + maxY) / 2;
    }
    
    return info;
}

// Sidestep LEFT
void sidestepLeft(WbMotionRef stepLeft) {
    if (!stepLeft) return;
    printf("  ⬅️ Sidestepping LEFT...\n");
    wbu_motion_play(stepLeft);
    for (int i = 0; i < 30; i++) {
        wb_robot_step(TIME_STEP);
        if (wbu_motion_is_over(stepLeft)) break;
    }
    wbu_motion_stop(stepLeft);
    wb_robot_step(TIME_STEP * 2);
}

// Sidestep RIGHT
void sidestepRight(WbMotionRef stepRight) {
    if (!stepRight) return;
    printf("  ➡️ Sidestepping RIGHT...\n");
    wbu_motion_play(stepRight);
    for (int i = 0; i < 30; i++) {
        wb_robot_step(TIME_STEP);
        if (wbu_motion_is_over(stepRight)) break;
    }
    wbu_motion_stop(stepRight);
    wb_robot_step(TIME_STEP * 2);
}

// Walk forward
void walkForward(WbMotionRef walk) {
    if (!walk) return;
    printf("  🚶 Walking forward...\n");
    wbu_motion_play(walk);
    for (int i = 0; i < 20; i++) {
        wb_robot_step(TIME_STEP);
        if (wbu_motion_is_over(walk)) {
            wbu_motion_play(walk);
        }
    }
    wbu_motion_stop(walk);
    wb_robot_step(TIME_STEP);
}

// Kick with left foot and log data
void kickWithLeftFoot(WbMotionRef kick, BallInfo ballInfo, int walkSteps, int sidestepDir) {
    if (!kick) return;
    
    // Save kick data BEFORE kicking
    lastKickData.timestamp = (int)time(NULL);
    lastKickData.ballSize = ballInfo.size;
    lastKickData.ballCenterX = ballInfo.centerX;
    lastKickData.ballCenterY = ballInfo.centerY;
    lastKickData.ballLeftEdge = ballInfo.leftEdge;
    lastKickData.ballRightEdge = ballInfo.rightEdge;
    lastKickData.redPixelCount = ballInfo.redPixels;
    lastKickData.walkStepsTaken = walkSteps;
    lastKickData.sidestepDirection = sidestepDir;
    sprintf(lastKickData.kickType, "LeftFootKick");
    
    // Print detailed kick data to console
    printf("\n═══════════════════════════════════════════════════\n");
    printf("📊 KICK DATA LOG (BEFORE KICK):\n");
    printf("═══════════════════════════════════════════════════\n");
    printf("  Timestamp: %d\n", lastKickData.timestamp);
    printf("  Ball Size: %d pixels\n", lastKickData.ballSize);
    printf("  Ball Center: (%d, %d)\n", lastKickData.ballCenterX, lastKickData.ballCenterY);
    printf("  Ball Edges: Left=%d, Right=%d\n", lastKickData.ballLeftEdge, lastKickData.ballRightEdge);
    printf("  Ball Width: %d pixels\n", lastKickData.ballRightEdge - lastKickData.ballLeftEdge);
    printf("  Red Pixels: %d\n", lastKickData.redPixelCount);
    printf("  Walk Steps Taken: %d\n", lastKickData.walkStepsTaken);
    printf("  Sidestep Direction: %s\n", 
           sidestepDir == 1 ? "LEFT" : (sidestepDir == 2 ? "RIGHT" : "NONE"));
    printf("  Kick Type: %s\n", lastKickData.kickType);
    printf("═══════════════════════════════════════════════════\n");
    
    
    FILE *file = fopen("kick_data_log.txt", "a");
    if (file != NULL) {
        fprintf(file, "KICK_DATA: timestamp=%d, size=%d, centerX=%d, centerY=%d, "
                      "leftEdge=%d, rightEdge=%d, redPixels=%d, walkSteps=%d, sidestepDir=%d, kick=%s\n",
                      lastKickData.timestamp, lastKickData.ballSize, 
                      lastKickData.ballCenterX, lastKickData.ballCenterY,
                      lastKickData.ballLeftEdge, lastKickData.ballRightEdge,
                      lastKickData.redPixelCount, lastKickData.walkStepsTaken,
                      lastKickData.sidestepDirection, lastKickData.kickType);
        fclose(file);
        printf("\n💾 Kick data saved to 'kick_data_log.txt'\n");
    }
    
    // Execute the kick
    printf("\n⚽⚽⚽ KICKING WITH LEFT FOOT! ⚽⚽⚽\n");
    wbu_motion_play(kick);
    while (!wbu_motion_is_over(kick)) {
        wb_robot_step(TIME_STEP);
    }
    printf("✅ KICK COMPLETE!\n");
}

void adjustHead() {
    WbDeviceTag headPitch = wb_robot_get_device("HeadPitch");
    if (headPitch) {
        wb_motor_set_position(headPitch, 0.35);
    }
}

int main() {
    wb_robot_init();
    
    printf("\n========================================\n");
    printf("NAO Robot - Smart Sidestep Based on Ball Position\n");
    printf("========================================\n");
    printf("Sequence:\n");
    printf("  1. Walk forward until ball size = %d pixels\n", KICK_THRESHOLD);
    printf("  2. Check ball position (LEFT or RIGHT of center)\n");
    printf("  3. Sidestep in SAME direction as ball\n");
    printf("  4. Kick with left foot\n\n");
    
    // Initialize cameras
    WbDeviceTag camTop = wb_robot_get_device("CameraTop");
    WbDeviceTag camBottom = wb_robot_get_device("CameraBottom");
    
    if (!camTop || !camBottom) {
        printf("ERROR: Cameras not found!\n");
        return -1;
    }
    
    wb_camera_enable(camTop, TIME_STEP);
    wb_camera_enable(camBottom, TIME_STEP);
    printf("✓ Cameras ready\n");
    
    adjustHead();
    
    // Load motions
    WbMotionRef walk = wbu_motion_new("../../motions/Forwards.motion");
    WbMotionRef stepLeft = wbu_motion_new("../../motions/SideStepLeft.motion");
    WbMotionRef stepRight = wbu_motion_new("../../motions/SideStepRight.motion");
    WbMotionRef kick = wbu_motion_new("../../motions/LeftFootKick.motion");
    
    if (!kick) {
        printf("⚠️ LeftFootKick.motion not found, using Shoot.motion\n");
        kick = wbu_motion_new("../../motions/Shoot.motion");
    }
    
    if (!walk || !stepLeft || !stepRight || !kick) {
        printf("ERROR: Missing motion files!\n");
        return -1;
    }
    printf("✓ Motions loaded\n");
    
    // cameras stabilize
    for (int i = 0; i < 10; i++) {
        wb_robot_step(TIME_STEP);
    }
    
    // ===== PHASE 1: FIND BALL =====
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("PHASE 1: FINDING BALL\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    BallInfo ballInfo;
    int found = 0;
    for (int i = 0; i < 100; i++) {
        ballInfo = detectBallFromCamera(camTop);
        if (ballInfo.detected) {
            found = 1;
            printf("✅ Ball found! Size: %d pixels\n", ballInfo.size);
            break;
        }
        if (i % 20 == 0) printf("🔍 Searching...\n");
        wb_robot_step(TIME_STEP);
    }
    
    if (!found) {
        printf("❌ No red ball detected!\n");
        return -1;
    }
    
    // ===== PHASE 2: WALK UNTIL BALL SIZE = THRESHOLD =====
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("PHASE 2: WALKING UNTIL BALL SIZE = %d PIXELS\n", KICK_THRESHOLD);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    int walkSteps = 0;
    int targetReached = 0;
    
    // get ball into bottom camera
    printf("Getting ball into bottom camera...\n");
    for (int i = 0; i < 6; i++) {
        ballInfo = detectBallFromCamera(camBottom);
        if (ballInfo.detected && ballInfo.size > 10) {
            printf("✓ Ball in bottom camera! Size: %d pixels\n", ballInfo.size);
            break;
        }
        walkForward(walk);
        walkSteps++;
    }
    
    // Continue walking until ball size reaches threshold
    printf("\nWalking until ball size reaches %d pixels...\n", KICK_THRESHOLD);
    
    while (!targetReached && walkSteps < 25) {
        ballInfo = detectBallFromCamera(camBottom);
        
        if (!ballInfo.detected) {
            printf("⚠️ Lost ball! Continuing to walk...\n");
            walkForward(walk);
            walkSteps++;
            continue;
        }
        
        printf("  Current ball size: %d pixels", ballInfo.size);
        
        // Progress bar
        int progress = (ballInfo.size * 100) / KICK_THRESHOLD;
        if (progress > 100) progress = 100;
        printf(" [");
        for (int i = 0; i < 20; i++) {
            printf("%s", i < progress/5 ? "█" : "░");
        }
        printf("] %d%%\n", progress);
        
        if (ballInfo.size >= KICK_THRESHOLD) {
            printf("\n✅✅✅ BALL SIZE REACHED %d PIXELS! ✅✅✅\n", ballInfo.size);
            targetReached = 1;
            break;
        }
        
        walkForward(walk);
        walkSteps++;
    }
    
    if (!targetReached) {
        printf("\n⚠️ Ball size only reached %d pixels, but continuing...\n", ballInfo.size);
    }
    
    printf("\nTotal forward steps taken: %d\n", walkSteps);
    
    // ===== PHASE 3: CHECK BALL POSITION AND SIDESTEP ACCORDINGLY =====
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("PHASE 3: CHECKING BALL POSITION\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    // Get final ball position
    ballInfo = detectBallFromCamera(camBottom);
    if (!ballInfo.detected) {
        ballInfo = detectBallFromCamera(camTop);
    }
    
    int sidestepDirection = 0;  // 0=none, 1=left, 2=right
    int cameraCenter = CENTER_TARGET;
    
    if (ballInfo.detected) {
        int ballX = ballInfo.centerX;
        printf("\n📊 BALL POSITION ANALYSIS:\n");
        printf("   Ball Center X: %d\n", ballX);
        printf("   Camera Center: %d\n", cameraCenter);
        
        if (ballX > cameraCenter) {
            printf("   Ball is on the RIGHT side of center\n");
            printf("\n🎯 Sidestepping RIGHT to align ball for left foot...\n");
            sidestepRight(stepRight);
            sidestepDirection = 2;  // RIGHT
        } 
        else if (ballX < cameraCenter) {
            printf("   Ball is on the LEFT side of center\n");
            printf("\n🎯 Sidestepping LEFT to align ball for left foot...\n");
            sidestepLeft(stepLeft);
            sidestepDirection = 1;  // LEFT
        }
        else {
            printf("   Ball is PERFECTLY CENTERED!\n");
            printf("\n✅ No sidestep needed!\n");
            sidestepDirection = 0;  // NONE
        }
    } else {
        printf("⚠️ Cannot detect ball position!\n");
    }
    
    // Wait after sidestep
    if (sidestepDirection != 0) {
        wb_robot_step(TIME_STEP * 2);
        printf("✅ Sidestep complete!\n");
        
        // Show new ball position
        ballInfo = detectBallFromCamera(camBottom);
        if (ballInfo.detected) {
            printf("   New ball position: X = %d\n", ballInfo.centerX);
        }
    }
    
    // ===== PHASE 4: LOG DATA AND KICK =====
    printf("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("PHASE 4: LOGGING DATA & KICKING\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    
    // Final ball check for kick data
    ballInfo = detectBallFromCamera(camBottom);
    if (!ballInfo.detected) {
        ballInfo = detectBallFromCamera(camTop);
    }
    
    // Stop all motion before kicking
    wbu_motion_stop(walk);
    wb_robot_step(TIME_STEP * 2);
    
    // Kick with left foot and log data
    kickWithLeftFoot(kick, ballInfo, walkSteps, sidestepDirection);
    
    printf("\n========================================\n");
    printf("          🎉 SUCCESS! 🎉\n");
    printf("========================================\n");
    printf("Sequence completed:\n");
    printf("  ✅ Walked %d steps until ball size = %d pixels\n", walkSteps, KICK_THRESHOLD);
    printf("  ✅ Ball was %s of center\n", 
           sidestepDirection == 1 ? "LEFT" : (sidestepDirection == 2 ? "RIGHT" : "CENTER"));
    printf("  ✅ Sidestepped %s\n",
           sidestepDirection == 1 ? "LEFT" : (sidestepDirection == 2 ? "RIGHT" : "NOTHING"));
    printf("  ✅ Kicked with left foot\n");
    printf("========================================\n");
    
    // Print data for immediate use
    printf("\n📋 DATA FOR INTEGRATION:\n");
    printf("   ball_size = %d\n", lastKickData.ballSize);
    printf("   ball_center_x = %d\n", lastKickData.ballCenterX);
    printf("   ball_left_edge = %d\n", lastKickData.ballLeftEdge);
    printf("   ball_right_edge = %d\n", lastKickData.ballRightEdge);
    printf("   walk_steps = %d\n", lastKickData.walkStepsTaken);
    printf("   sidestep_direction = %s\n",
           lastKickData.sidestepDirection == 1 ? "LEFT" : 
           (lastKickData.sidestepDirection == 2 ? "RIGHT" : "NONE"));
    printf("   optimal_distance_pixels = %d\n", KICK_THRESHOLD);
    
    // Cleanup
    wbu_motion_delete(walk);
    wbu_motion_delete(stepLeft);
    wbu_motion_delete(stepRight);
    wbu_motion_delete(kick);
    wb_robot_cleanup();
    
    return 0;
}