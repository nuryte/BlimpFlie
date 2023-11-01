import pygame
import time


class KeyboardHandler:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))  # Increased screen size to display controls
        pygame.display.set_caption("Keyboard Controls")
        
        # Displaying the controls
        self.display_controls()
        # Mapping keys to attributes
        self.button_map = {
            pygame.K_UP: "x",
            pygame.K_DOWN: "y",
            pygame.K_LEFT: "a",
            pygame.K_RIGHT: "b",
            pygame.K_r: "lb",
            pygame.K_t: "rb"
        }

        # Initialize state attributes
        for btn in self.button_map.values():
            setattr(self, btn, 0)
            setattr(self, f"{btn}_old", 0)
            setattr(self, f"{btn}_state", 0)

        self.right_vertical, self.right_horizontal, self.left_vertical, self.left_horizontal = 0, 0, 0, 0
        self.time_start = time.time()

    def update_key_params(self):
        """Update keyboard parameters."""
        pygame.event.pump()
        
        keys = pygame.key.get_pressed()

        # Update button values and states
        for key, btn in self.button_map.items():
            current_val = 1 if keys[key] else 0
            old_val = getattr(self, f"{btn}_old")
            
            if current_val and not old_val:  # Check for rising edge
                setattr(self, f"{btn}_state", not getattr(self, f"{btn}_state"))
            
            setattr(self, btn, current_val)
            setattr(self, f"{btn}_old", current_val)

        # Simulating right joystick with WASD keys
        self.right_vertical = -1 if keys[pygame.K_w] else 1 if keys[pygame.K_s] else 0
        self.right_horizontal = -1 if keys[pygame.K_a] else 1 if keys[pygame.K_d] else 0

        # Simulating left joystick with Shift/Space and Q/E
        self.left_vertical = 1 if keys[pygame.K_SPACE] else -1 if keys[pygame.K_LSHIFT] else 0
        self.left_horizontal = -1 if keys[pygame.K_q] else 1 if keys[pygame.K_e] else 0
    
    def display_controls(self):
        """Display the keyboard controls on the pygame window."""
        font = pygame.font.SysFont(None, 25)
        controls = {
            "Arrow UP": "X",
            "Arrow DOWN": "Y",
            "Arrow LEFT": "A",
            "Arrow RIGHT": "B",
            "R": "LB",
            "T": "RB",
            "WASD": "Right Joystick",
            "Shift/Space": "Left Vertical",
            "Q/E": "Left Horizontal",
            "ESC": "Exit"
        }
        
        self.screen.fill((255, 255, 255))
        y_offset = 10
        for key, action in controls.items():
            text = font.render(f"{key} : {action}", True, (0, 0, 0))
            self.screen.blit(text, (10, y_offset))
            y_offset += 30

        pygame.display.flip()

    def get_outputs(self):
        """Get the output controls based on key states."""
        self.update_key_params()

        # This part remains the same as your joystick handler logic.
        # Adjust accordingly if you want to change the behavior.
        #if self.a == "bicopter":
        return self.get_bicopter_controls()

    def get_bicopter_controls(self):
        """Return controls for bicopter."""
        dt = time.time() - self.time_start
        self.time_start = time.time()

        self.fx = -1* self.right_vertical
        self.fz = self.fz + self.left_vertical * dt if self.lb_state else 0
        self.tz = self.right_horizontal
        return [self.lb_state, self.fx, 0, self.fz, 0, 0, self.tz, 0, 0, 0, 0, 0], self.y_state

if __name__ == "__main__":
    handler = KeyboardHandler()

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    pygame.quit()
                    exit()

            outputs = handler.get_outputs()
            print(outputs)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Loop terminated by user.")
