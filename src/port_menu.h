/* Shared Windows display menu; included after the SDL renderer declarations. */
#if defined(_WIN32) && !defined(SNESRECOMP_PORT_MENU_H)
#define SNESRECOMP_PORT_MENU_H
static void PortDisplayMenu(void) {
  SDL_SysWMinfo info;
  SDL_VERSION(&info.version);
  if (!SDL_GetWindowWMInfo(g_window, &info)) return;
  HMENU menu = CreatePopupMenu(), aspect = CreatePopupMenu(), scale = CreatePopupMenu(), filter = CreatePopupMenu();
  AppendMenuA(menu, MF_STRING, 1, "Fullscreen / windowed");
  AppendMenuA(aspect, MF_STRING, 2, "Original pixels (8:7)");
  AppendMenuA(aspect, MF_STRING, 3, "Classic TV (4:3)");
  AppendMenuA(aspect, MF_STRING, 4, "Widescreen (16:9 stretched)");
  AppendMenuA(menu, MF_POPUP, (UINT_PTR)aspect, "Picture shape");
  for (int i = 1; i <= 5; i++) {
    char label[40]; snprintf(label, sizeof(label), "%dx window size", i);
    AppendMenuA(scale, MF_STRING, 10 + i, label);
  }
  AppendMenuA(menu, MF_POPUP, (UINT_PTR)scale, "Size / upscaling");
  AppendMenuA(filter, MF_STRING, 20, "Sharp pixels");
  AppendMenuA(filter, MF_STRING, 21, "Smooth pixels");
  AppendMenuA(menu, MF_POPUP, (UINT_PTR)filter, "Filter");
  POINT p; GetCursorPos(&p);
  int command = TrackPopupMenu(menu, TPM_RETURNCMD | TPM_NONOTIFY, p.x, p.y, 0, info.info.win.window, NULL);
  DestroyMenu(menu);
  if (command == 1) {
    g_win_flags ^= SDL_WINDOW_FULLSCREEN_DESKTOP;
    SDL_SetWindowFullscreen(g_window, g_win_flags & SDL_WINDOW_FULLSCREEN_DESKTOP);
  } else if (command >= 2 && command <= 4 && g_renderer) {
    g_config.ignore_aspect_ratio = false;
    SDL_RenderSetLogicalSize(g_renderer, command == 2 ? 256 : command == 3 ? 896 : 1792,
                            command == 2 ? 224 : command == 3 ? 672 : 1008);
  } else if (command >= 11 && command <= 15) {
    SDL_SetWindowFullscreen(g_window, 0);
    g_win_flags &= ~SDL_WINDOW_FULLSCREEN_DESKTOP;
    SDL_SetWindowSize(g_window, 256 * (command - 10), 224 * (command - 10));
  } else if ((command == 20 || command == 21) && g_texture) {
    g_config.linear_filtering = command == 21;
    SDL_SetTextureScaleMode(g_texture, command == 21 ? SDL_ScaleModeLinear : SDL_ScaleModeNearest);
  }
}
#endif
