#include <iostream>
#include <pgm.h>
#include <SDL2/SDL.h>
#include <draw.h>
#include <config.h>
#include <error.h>
#include <observer.h>
#include <rrt_planner/rrt_planner.h>
#include <sys/time.h>

long long current_time() {
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}

#define WIDTH 640
#define HEIGHT 480
#define FPS 30
#define ZOOM 3

#define TRIES 10
#define AUTO

double times[TRIES];
unsigned times_aux = 0;

int main(int argc, char* argv[])
{
  std::string config_name;

  if (argc != 2) {
    config_name = "param/default-1.json";
  } else {
    config_name = argv[1];
  }

  config_t config = get_config(config_name);
  print_config(config);

  const pgm_t pgm = parsePGM(config.map_file);

  std::vector<unsigned int> pixels = pgm.pixels;
  const int width = pgm.width;
  const int height = pgm.height; 

  /* Initializes the timer, audio, video, joystick,
  haptic, gamecontroller and events subsystems */
  if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
  {
    printf("Error initializing SDL: %s\n", SDL_GetError());
    return 0;
  }
  /* Create a window */
  SDL_Window* window = SDL_CreateWindow("View", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width*ZOOM, height*ZOOM, SDL_WINDOW_SHOWN);

  if (!window)
  {
    printf("Error creating window: %s\n", SDL_GetError());
    SDL_Quit();
    return 0;
  }
  /* Create a renderer */
  Uint32 render_flags = SDL_RENDERER_ACCELERATED;
  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, render_flags);


  if (!renderer)
  {
    printf("Error creating renderer: %s\n", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
  }

  SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STATIC, width, height);
  if (!texture) {
    std::cerr << "Texture creation failed: " << SDL_GetError() << std::endl;
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  /* Main loop */
  bool running = true;
  SDL_Event event;

  /* Clear screen */
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);


  /* RRT Planner */

  double start[2] = {config.initial_x, config.initial_y}, goal[2] = {config.goal_x, config.goal_y};

  const rrt_planner::rrt_params params = config_to_params(config);
  rrt_planner::RRTPlanner planner(&pgm, params);

  planner.setStart(start);
  planner.setGoal(goal);
  RRTObserver observer(renderer, &planner, ZOOM);

  bool path_found = false;
  bool reached_goal = false;

  auto initial_time = current_time();

  while (running)
  {
    if (!path_found) {
      path_found = planner.planPath();
      reached_goal = planner.reached_goal;

      if (path_found) {
        if (reached_goal) {
#ifndef AUTO
          SUCCESS("%f", (double) (current_time() - initial_time) / 1000.0);
        }
#else
          times[times_aux++] = (double) (current_time() - initial_time) / 1000.0;
          if(times_aux == TRIES) {
            double sum = 0;
            for (int i = 0; i < TRIES; i++) {
              printf("%3f\n", times[i]);
              sum += times[i];
            }
            printf("---\n");
            printf("%3f\n", sum / TRIES);
            exit(0);
          }
          path_found = false; 
          reached_goal = false;
          planner.setStart(start);
          initial_time = current_time();
      } else {
          planner.move();
          path_found = false; 
          reached_goal = false;
      }
#endif
      }
    }

    /* Process events */
    while (SDL_PollEvent(&event))
    {
      switch (event.type)
      {
        case SDL_QUIT:
          running = false;
          break;
        case SDL_KEYDOWN:
          switch (event.key.keysym.scancode)
          {
            case SDL_SCANCODE_SPACE:
              break;
            case SDL_SCANCODE_R:
              if (path_found) {
                if (!reached_goal) {
                  planner.move();
                } else {
                  planner.setStart(start);
                  initial_time = current_time();
                }
                path_found = false; 
                reached_goal = false; 
              }
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
    }

    SDL_UpdateTexture(texture, nullptr, pixels.data(), width * sizeof(int));
    SDL_Rect dest_rect = {0, 0, width * ZOOM, height * ZOOM};
    SDL_RenderCopy(renderer, texture, NULL, &dest_rect);

    // DRAW STARTING POINT
    unsigned mx, my;
    pgm.worldToMap(planner.start_[0], planner.start_[1], mx, my);
    SDL_SetRenderDrawColor(renderer, 20, 200, 155, 255);
    sdl_circle(renderer, mx*ZOOM, my*ZOOM, 6);


    // DRAW GOAL
    pgm.worldToMap(config.goal_x, config.goal_y, mx, my);
    SDL_SetRenderDrawColor(renderer, 200, 10, 120, 255);
    sdl_circle(renderer, mx*ZOOM, my*ZOOM, ZOOM * config.goal_tolerance / RESOLUTION, false);

    observer.draw(&pgm);

    SDL_RenderPresent(renderer);
    SDL_Delay(1000/FPS);
  }

  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
