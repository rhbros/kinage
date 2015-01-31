#include <stdio.h>
#include <SDL/SDL.h>
#include <stdbool.h>
#include "core/arm7tdmi.h"

SDL_Surface* screen;
uint32_t* buffer;

void setpixel(int x, int y, int c)
{
    buffer[y * 240 + x] = c;
    SDL_UpdateRect(screen, x, y, 1, 1);
}

int32_t getcolor(int n, int p)
{
    uint16_t v = arm7_readh(0x05000000 + p * 32 + n * 2);
    uint32_t r = 0xFF000000;
    r |= ((v & 0x1f) * 8) << 16;
    r |= (((v >> 5) & 0x1f) * 8) << 8;
    r |= ((v >> 10) & 0x1f) * 8;
    return r;
}

void drawtile(int x, int y, int n, int p)
{
    int x2 = 0;
    int y2 = 0;
    for (int i = 0; i < 32; i++)
    {
        uint8_t b = arm7_readb(0x06004000 + 0x20 * n + i);
        if (x2 == 8)
        {
            x2 = 0;
            y2++;
        }
        setpixel( x + x2, y + y2, getcolor(b & 0xF, p) );
        setpixel( x + x2 + 1, y + y2, getcolor(b >> 4, p));
        x2 += 2;
    }
}

int main(int argc, char** argv)
{
    SDL_Event event;
    bool running = true;
    
    if (argc != 2)
    {
    	puts("Usage: fba rom.gba");
    	return 1;
    }
    
    mmu_setup(argv[1]);
    arm7_reset();

    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
        printf("SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }

    screen = SDL_SetVideoMode(240, 160, 32, SDL_SWSURFACE);
    if (screen == NULL)
    {
        printf("SDL_SetVideoMode Error: %s\n", SDL_GetError());
        SDL_Quit();
    }
    buffer = (uint32_t*)screen->pixels;

    SDL_WM_SetCaption("kinage", "kinage");

    while (running)
    {
        for (int i = 0; i < 10000000; i++)
            arm7_step();
        //ERROR("Drawing")
        for (int y = 0; y < 160; y++)
        {
            for (int x = 0; x < 240; x++)
            {
                uint32_t color = arm7_readb(0x06000000 + y * 240 + x);
                uint32_t argb = getcolor(color & 0xF, color >> 4);
                setpixel(x, y, argb);
            }
        }
        /*for (int y = 0; y < 20; y++)
        {
            for (int x = 0; x < 30; x++)
            {
                uint16_t v = arm7_readh(0x06000800 + y * 0x40 + x * 2);
                uint32_t p = v >> 8;
                uint32_t t = v & 0xFF;
                drawtile(x * 8, y * 8, t, p);
            }
        }*/

        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
                running = false;
        }
    }

    SDL_FreeSurface(screen);

    return 0;
}
