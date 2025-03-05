import pygame

def show_text(text, x, y):#專門顯示文字的方法，除了顯示文字還能指定顯示的位置
    x = x
    y = y
    text = font.render(text, True, (255, 255, 255))
    screen.blit(text, (x, y))
    pygame.display.update()

pygame.init()

version = " v0.0.0"
pygame.display.set_caption("test" + version)

clock = pygame.time.Clock()

screen = pygame.display.set_mode((800, 600))

background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((125, 125, 125))
screen.blit(background, (0, 0))

font = pygame.font.Font("msjh.ttc", 24)#本文主角

pygame.display.update()

running = True
run_opening = True
while running:
    
    clock.tick(30)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
    if run_opening:
        op_background = pygame.Surface(screen.get_size())
        op_background = op_background.convert()
        op_background.fill((0, 0, 0))
        screen.blit(op_background, (0, 0))
        
        text = "中文中文abc123"#文字顯示內容在此，根本用不著u
        show_text(text, 100, 0)#使用show_text()
        
        pygame.display.update()
        
pygame.quit()