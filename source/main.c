#define _CRT_SECURE_NO_WARNINGS

#pragma comment(lib, "SDL3.lib")

#include <SDL3/SDL.h>
#include "Nes_Cpu6502.h"
#include "Canvas.h"
#include "Fonts.h"

uint8_t _demo_memory[0x10000] = { 0 };
const char *_program = "A2 0A 8E 00 00 A2 03 8E 01 00 AC 00 00 A9 00 18 6D 01 00 88 D0 FA 8D 02 00 EA EA EA";

void load_program_to_memory(const char *program, uint16_t offset, uint8_t *memory) {
    int index = 0;
    while (*program != '\0') {
        char *endp = NULL;
        long byte = strtol(program, &endp, 16);
        program = endp;

        memory[offset + index++] = byte;
    }
}

void draw_demo_nes_cpu6502(Canvas *canvas, Nes_Cpu6502 *cpu, uint8_t *memory) {
    draw_nes_cpu6502_regs(canvas, 0, canvas->height - 8 * CANVAS_FONT_HEIGHT, cpu);
    // canvas->color = CANVAS_WHITE;
    // canvas_printf(canvas, 0, canvas->height - CANVAS_FONT_HEIGHT, " clk: %llu", cpu->clk_count);

    draw_nes_cpu6502_disasm(canvas, canvas->width - 22 * CANVAS_FONT_WIDTH, 0, 16, cpu, memory);

    draw_nes_cpu6502_memory(canvas, 0, 0, 16, 16, cpu, memory, 0x0000);
    draw_nes_cpu6502_memory(canvas, 0, 17 * CANVAS_FONT_HEIGHT, 16, 16, cpu, memory, 0x0100);
    draw_nes_cpu6502_memory(canvas, 0, 34 * CANVAS_FONT_HEIGHT, 16, 16, cpu, memory, 0x8000);

    canvas->color = CANVAS_LTGRAY;
    canvas_printf(canvas, 0, canvas->height - CANVAS_FONT_HEIGHT, "SPACE=step\tR=reset");
}

int main() {
    Nes_Cpu6502 nes_cpu6502 = nes_cpu6502_init(0x8000);
    memory_init(_demo_memory, 0x8000, 0x8000, 0x8000);
    load_program_to_memory(_program, 0x8000, _demo_memory);

    Canvas canvas = canvas_alloc(640, 480);
    // canvas_set_font(&canvas, _font_3dfx8x8);
    // canvas_set_font(&canvas, _font_ibm5155_cga_rom_memotek_gr_01__8x8);

    if (!SDL_Init(SDL_INIT_VIDEO))
        ASSERT(false, "%s", SDL_GetError());

    SDL_WindowFlags window_flags = 0;
    SDL_Window *window = SDL_CreateWindow("TEST KANKS", 640 * 2, 480 * 2, window_flags);
    ASSERT(window != NULL, "%s", SDL_GetError());

    SDL_Renderer *renderer = SDL_CreateRenderer(window, NULL);
    ASSERT(renderer != NULL, "%s", SDL_GetError());
    SDL_SetRenderVSync(renderer, 0);

    SDL_Texture *texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_ABGR8888,
        SDL_TEXTUREACCESS_STREAMING,
        canvas.width,
        canvas.height);
    ASSERT(texture != NULL, "%s", SDL_GetError());
    SDL_SetTextureScaleMode(texture, SDL_SCALEMODE_NEAREST);

    SDL_SetHint(SDL_HINT_KEYCODE_OPTIONS, "hide_numpad");
    SDL_StartTextInput(window);

    const uint64_t perf_freq = SDL_GetPerformanceFrequency();
    uint64_t curr_time = SDL_GetPerformanceCounter();
    uint64_t prev_time = curr_time;
    float delta_time = 0.0f;

    bool done = false;
    SDL_Event event = { 0 };
    while (!done) {
        curr_time = SDL_GetPerformanceCounter();
        delta_time = (float)(curr_time - prev_time) / perf_freq;
        prev_time = curr_time;

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT)
                done = true;
            if (event.type == SDL_EVENT_KEY_DOWN) {
                switch (event.key.key) {
                case SDLK_SPACE:
                    nes_cpu6502_step(&nes_cpu6502, _demo_memory);
                    break;
                case SDLK_R:
                    nes_cpu6502_intr_rst(&nes_cpu6502, _demo_memory);
                    break;
                default:
                    break;
                }
            }
        }

        canvas.color = JBLOW_BACKGROUND;
        canvas_fill(&canvas);
        // draw_font_bitmap(&canvas, 0, 0);
        // draw_memory(&canvas, 10, 0, 16, 48, main);

        draw_demo_nes_cpu6502(&canvas, &nes_cpu6502, _demo_memory);

        static int fps = 0;
        count_fps(delta_time, &fps);
        canvas.color = CANVAS_WHITE;
        canvas_printf(&canvas,
            canvas.width - 20 * CANVAS_FONT_WIDTH,
            canvas.height - 2 * CANVAS_FONT_HEIGHT,
            "dt:\t%.5f\nfps:\t%d", delta_time, fps);

        SDL_UpdateTexture(texture, NULL, canvas.buffer, canvas.pitch);
        SDL_RenderTexture(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);
        // SDL_Delay(500);
    }

    SDL_StopTextInput(window);

    canvas_free(&canvas);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return EXIT_SUCCESS;
}