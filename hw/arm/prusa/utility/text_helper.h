// A quick and dirty helper for rendering characters. 
// Lifted from console.c


#define FONT_HEIGHT 16
#define FONT_WIDTH 8

typedef struct QemuConsole QemuConsole;

#define QEMU_RGB(r, g, b)                                               \
    { .red = r << 8, .green = g << 8, .blue = b << 8, .alpha = 0xffff }


void vga_putcharxy(QemuConsole *s, int x, int y, int ch,
                        pixman_color_t colours[2]);

void vga_putcharxy_s(QemuConsole *s, int x, int y, int ch,
                        pixman_color_t* fg, pixman_color_t* bg);
