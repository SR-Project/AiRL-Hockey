import pygame
import numpy as np
import cv2
import os

from air_hockey_game.src.dimensions import Dimensions
from air_hockey_game.src.circle import Puck, Mallet, Target
from air_hockey_game.src.force import ForceRegistry, ControlledForce
from air_hockey_game.src.collision import Collision
from air_hockey_game.src.ai import AI
from air_hockey_game.src.line import Line
from air_hockey_game.src.score import Score
from air_hockey_game.src.game_info import GameInfo
from air_hockey_game.src.sprite_utils import load_sprites, blit_puck
import air_hockey_game.src.vector as V
import air_hockey_game.src.phy_const as P


default_use_object={'puck': True, 'top_mallet': True}

class AirHockey(object):
    def __init__(self, dim=Dimensions(), video_file=None, render=False):

        self.render = render

        self.dim = dim

        # Add Walls
        # Arcs and Lines have to be passed in an anti-clockwise order with respect to the self.dim.center
        self.top_wall          = Line(self.dim.arc_top_left_start, self.dim.arc_top_right_end)
        self.bottom_wall       = Line(self.dim.arc_bottom_left_end, self.dim.arc_bottom_right_start)
        self.left_wall         = Line(self.dim.arc_top_left_end, self.dim.arc_bottom_left_start)
        self.right_wall        = Line(self.dim.arc_top_right_start, self.dim.arc_bottom_right_end)

        self.top_left_wall     = Line(self.dim.arc_top_left_start, self.dim.post_top_left)
        self.top_right_wall    = Line(self.dim.post_top_right, self.dim.arc_top_right_end)
        self.bottom_left_wall  = Line(self.dim.arc_bottom_left_end, self.dim.post_bottom_left)
        self.bottom_right_wall = Line(self.dim.post_bottom_right, self.dim.arc_bottom_right_start)

        self.center_line       = Line(self.dim.center_left, self.dim.center_right)

        # Add Corners
        self.top_left_corner     = Line.generate_bezier_curve(self.dim.arc_top_left, self.dim.bezier_ratio)
        self.top_right_corner    = Line.generate_bezier_curve(self.dim.arc_top_right, self.dim.bezier_ratio)
        self.bottom_left_corner  = Line.generate_bezier_curve(self.dim.arc_bottom_left, self.dim.bezier_ratio)
        self.bottom_right_corner = Line.generate_bezier_curve(self.dim.arc_bottom_right, self.dim.bezier_ratio)

        self.borders = [self.top_left_wall, self.top_right_wall, self.bottom_left_wall, self.bottom_right_wall, self.left_wall, self.right_wall] + \
                        self.top_left_corner + self.top_right_corner + self.bottom_left_corner + self.bottom_right_corner

        self.puck = Puck(self.dim.center, self.dim.puck_radius, self.borders)

        self.top_mallet = Mallet(self.dim.top_mallet_position, self.dim.mallet_radius,
                          [self.top_wall, self.center_line, self.left_wall, self.right_wall] + self.top_left_corner + self.top_right_corner)

        self.bottom_mallet = Mallet(self.dim.bottom_mallet_position, self.dim.mallet_radius,
                          [self.center_line, self.bottom_wall, self.left_wall,self. right_wall] + self.bottom_left_corner + self.bottom_right_corner)

        self.bodies = [self.puck, self.top_mallet, self.bottom_mallet]
        
        self.bottom_target = Target([self.dim.center[0], self.dim.rink_bottom - 55], self.dim.target_radius)

        self.top_ai    = AI(self.top_mallet,    self.puck, mode='top',    dim=self.dim)
        self.bottom_ai = AI(self.bottom_mallet, self.puck, mode='bottom', dim=self.dim)

        self.top_ai_force    = ControlledForce()
        self.bottom_ai_force = ControlledForce()

        self.forces = ForceRegistry()
        self.forces.add(self.top_mallet,    self.top_ai_force)
        self.forces.add(self.bottom_mallet, self.bottom_ai_force)

        self.score = Score(self.dim)

        # Initialize Pygame only if rendering is enabled
        if self.render:
            pygame.init()
            self.screen = pygame.display.set_mode((self.dim.width, self.dim.height))
            self.font = pygame.font.SysFont("monospace", 30)
            pygame.display.set_caption('Air Hockey')

        # # Initialize pygame variables
        # pygame.init()
        # self.screen = pygame.display.set_mode((self.dim.width, self.dim.height))
        # self.font = pygame.font.SysFont("monospace", 30)
        # pygame.display.set_caption('Air Hockey')

        # Initialize a video writer
        self.writer = None
        if video_file is not None:
            if os.path.isfile(video_file): os.remove(video_file)
            self.writer = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'PIM1'), 30,
                          (self.dim.width, self.dim.height-self.dim.vertical_margin*2))

        # Allocate memory for frame
        self.frame = np.zeros((self.dim.width, self.dim.height, 3), dtype=np.uint8)
        self.cropped_frame = np.zeros((self.dim.height-2*self.dim.vertical_margin, self.dim.width, 3), dtype=np.uint8)
        
        self.distance = P.max_distance

        self.reset()

    
    def _draw_table(self):
        # Draw borders (replace with actual coordinates from self.dim)
        pygame.draw.line(self.screen, (255, 255, 255), 
                        self.dim.arc_top_left_start.astype(int).tolist(), 
                        self.dim.arc_top_right_end.astype(int).tolist(), 5)
                        
        pygame.draw.line(self.screen, (255, 255, 255), 
                        self.dim.arc_bottom_left_end.astype(int).tolist(), 
                        self.dim.arc_bottom_right_start.astype(int).tolist(), 5)
                        
        pygame.draw.line(self.screen, (255, 255, 255), 
                        self.dim.arc_top_left_end.astype(int).tolist(), 
                        self.dim.arc_bottom_left_start.astype(int).tolist(), 5)
                        
        pygame.draw.line(self.screen, (255, 255, 255), 
                        self.dim.arc_top_right_start.astype(int).tolist(), 
                        self.dim.arc_bottom_right_end.astype(int).tolist(), 5)

        # Draw corners (curves)
        for curve in self.top_left_corner + self.top_right_corner + self.bottom_left_corner + self.bottom_right_corner:
            pygame.draw.line(self.screen, (255, 255, 255), 
                            curve.p1.astype(int).tolist(), 
                            curve.p2.astype(int).tolist(), 5)

        # Draw center line
        pygame.draw.line(self.screen, (255, 255, 255), 
                        self.dim.center_left.astype(int).tolist(), 
                        self.dim.center_right.astype(int).tolist(), 5)

        # Draw goals (as rectangles or lines)
        pygame.draw.line(self.screen, (0, 255, 0), 
                        self.dim.post_top_left.astype(int).tolist(), 
                        self.dim.post_top_right.astype(int).tolist(), 5)
                        
        pygame.draw.line(self.screen, (0, 255, 0), 
                        self.dim.post_bottom_left.astype(int).tolist(), 
                        self.dim.post_bottom_right.astype(int).tolist(), 5)
        
    def _draw_circles(self, puck, top_mallet, bottom_mallet):
        # draw top_mallet
        pygame.draw.circle(self.screen, (255, 0, 0), (top_mallet).tolist(), self.dim.mallet_radius)
        
        # draw bottom_mallet
        pygame.draw.circle(self.screen, (0, 0, 255), (bottom_mallet).tolist(), self.dim.mallet_radius)

        # draw puck
        pygame.draw.circle(self.screen, (255, 255, 255), puck.tolist(), self.dim.puck_radius)

    def _draw(self, puck, top_mallet, bottom_mallet, debug=False, use_object=default_use_object):
        
        draw_puck = use_object['puck']
        draw_top_mallet = use_object['top_mallet']
        
        # Background color
        self.screen.fill((0, 0, 0))
        self._draw_table()
        
        # Draw dark blue target to which the robot mallet should when the puck is on the opposite side
        pygame.draw.circle(self.screen, (0,0,139), self.bottom_target.position.tolist(), self.bottom_target.radius)
        
        # if draw_top_mallet:
        #     self.screen.blit(self.sprites['top_mallet'],    (top_mallet - self.dim.mallet_radius).tolist())
        
        # self.screen.blit(self.sprites['bottom_mallet'], (bottom_mallet - self.dim.mallet_radius).tolist())

        # if draw_puck:
        #     blit_puck(self, puck)

        if self.writer:
            self.writer.write(self.cropped_frame[:,:,::-1])

        if debug:
            for line in self.borders:
                pygame.draw.line(self.screen, (0, 255, 255), line.p2, line.p1, 6)

        self._draw_circles(puck, top_mallet, bottom_mallet)

    def _render(self, debug=False, use_object=default_use_object):
        if not self.render:
            return
        
        self._draw(self.puck.position, self.top_mallet.position, self.bottom_mallet.position, debug, use_object)
        self.screen.blit(self.font.render('%4d' % self.score.get_top(),    1, (200, 0, 0)), (0, 30))
        self.screen.blit(self.font.render('%4d' % self.score.get_bottom(), 1, (0, 200, 0)), (0, self.dim.rink_bottom+30))

        pygame.display.update()

        self.frame[:] = pygame.surfarray.array3d(self.screen)
        self.cropped_frame[:] = self.frame[:, self.dim.vertical_margin:-self.dim.vertical_margin, :].transpose((1,0,2))

    def reset(self,
              scored=False,
              puck_was_hit=False,
              puck_is_at_the_bottom=False,
              distance_decreased=False,
              hit_the_border=False,
              use_object=default_use_object):

        # self.puck.reset(self.dim, self.dim.rink_top, self.dim.rink_bottom)
        self.puck.reset(self.dim, self.dim.rink_top, self.dim.center[1])
        self.top_mallet.reset(self.dim, self.dim.rink_top, self.dim.center[1])
        self.bottom_mallet.reset(self.dim, self.dim.center[1], self.dim.rink_bottom)

        # Resolve possible interpenetration
        Collision.circle_circle([self.puck, self.top_mallet])
        Collision.circle_circle([self.top_mallet, self.bottom_mallet])
        
        in_the_target = Collision.circle_circle([self.bottom_mallet, self.bottom_target], resolve=False)

        self.sprites = load_sprites()

        self._render(use_object=use_object)

        puck_info = [self.puck.position, self.puck._velocity]
        mallet_info = [self.bottom_mallet.position, self.bottom_mallet._velocity]
        return GameInfo(self.cropped_frame,
                        puck_info,
                        mallet_info,
                        scored=scored,
                        puck_was_hit=puck_was_hit,
                        puck_is_at_the_bottom=puck_is_at_the_bottom,
                        distance_decreased=distance_decreased,
                        hit_the_border=hit_the_border,
                        in_the_target=in_the_target)

    def step(self,
             robot_action=None,
             human_action=None,
             debug=False,
             use_object=default_use_object,
             n_steps=4):

        dt = np.random.ranf() + 1 # dt is randomly in interval [1, 2)

        if robot_action is not None:
            if not isinstance(robot_action, np.ndarray):
                raise Exception('Action is supposed to be a numpy array')
            elif robot_action.shape[0] != 2:
                raise Exception('Action array can only have 2 values (x and y)')
            elif robot_action.min() < -1 or robot_action.max() > 1:
                raise Exception('Values of x and y have to be in range [-1, 1]')

        if human_action is not None:
            if not isinstance(human_action, np.ndarray):
                raise Exception('Adversarial action is supposed to be a numpy array')
            elif human_action.shape[0] != 2:
                raise Exception('Adversarial action array can only have 2 values (x and y)')
            elif human_action.min() < -1 or human_action.max() > 1:
                raise Exception('Adversarial values of x and y have to be in range [-1, 1]')

        if robot_action is None:
            robot_action = self.bottom_ai.move()

        if human_action is None:
            human_action = self.top_ai.move()

        # Update forces
        self.top_ai_force.set_force(human_action)
        self.bottom_ai_force.set_force(robot_action)
        
        for _ in range(n_steps):

            # Clear forces from last frame
            for body in self.bodies:
                body.clear_accumulators()
            self.forces.update_forces()
    
            # Move bodies
            for body in self.bodies:
                body.integrate(dt)
    
            # Check collisions between all possible pairs of bodies
            
            if use_object['top_mallet']:
                Collision.circle_circle([self.puck, self.top_mallet])
            Collision.circle_circle([self.top_mallet, self.bottom_mallet])
            
            if use_object['puck']:
                puck_was_hit = Collision.circle_circle([self.puck, self.bottom_mallet])
            else:
                puck_was_hit = False
            
            in_the_target = Collision.circle_circle([self.bottom_mallet, self.bottom_target], resolve=False)
    
            # Make sure all bodies are within their borders
            collided = [False, False, False]
            for i, body in enumerate(self.bodies):
                for border in body.borders:
                    if (Collision.circle_line(body, border)):
                        collided[i] = True
            hit_the_border = collided[2]
    
            puck_is_at_the_bottom = self.puck.position[1] > self.dim.center[1]
    
            distance_decreased = False
            if puck_is_at_the_bottom:
                distance = V.magnitude(self.puck.position - self.bottom_mallet.position)
                distance_decreased = distance < self.distance
                self.distance = distance
            else:
                self.distance = P.max_distance
    
            scored = self.score.update(self.puck)
            if scored is not None:
                return self.reset(scored,
                                  puck_was_hit,
                                  puck_is_at_the_bottom,
                                  distance_decreased,
                                  hit_the_border)
    
            self._render(debug, use_object)

        puck_info = [self.puck.position, self.puck._velocity]
        mallet_info = [self.bottom_mallet.position, self.bottom_mallet._velocity]
        return GameInfo(self.cropped_frame,
                        puck_info,
                        mallet_info,
                        robot_action,
                        human_action,
                        scored,
                        puck_was_hit,
                        puck_is_at_the_bottom,
                        distance_decreased,
                        hit_the_border,
                        in_the_target)
        