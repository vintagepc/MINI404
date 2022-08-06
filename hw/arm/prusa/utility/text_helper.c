// A quick and dirty helper for rendering characters. 
// Lifted from console.c

#include "qemu/osdep.h"
#include "ui/console.h"

#include "ui/vgafont.h"
#include "text_helper.h"


extern void vga_putcharxy_s(QemuConsole *s, int x, int y, int ch,
                          pixman_color_t* fg, pixman_color_t* bg)
{
    static pixman_image_t *glyphs[256];
    DisplaySurface *surface = qemu_console_surface(s);
    
    if (!glyphs[ch]) {
        glyphs[ch] = qemu_pixman_glyph_from_vgafont(FONT_HEIGHT, vgafont16, ch);
    }
    qemu_pixman_glyph_render(glyphs[ch], surface->image,
                             fg, bg, x, y, FONT_WIDTH, FONT_HEIGHT);
}

extern void vga_putcharxy(QemuConsole *s, int x, int y, int ch,
                          pixman_color_t colours[2])
{
        vga_putcharxy_s(s, x, y, ch, &colours[0], &colours[1]);
}
