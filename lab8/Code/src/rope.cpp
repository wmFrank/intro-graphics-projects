#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        for(int i = 0; i < num_nodes; i++){
            masses.emplace_back(new Mass(start + i * (end - start) / (num_nodes - 1), node_mass, false));
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
        for(int j = 0; j < num_nodes - 1; j++){
            springs.emplace_back(new Spring(masses[j], masses[j + 1], k));
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            s->m1->forces += s->k * (s->m2->position - s->m1->position).unit() * ((s->m2->position - s->m1->position).norm() - s->rest_length);
            s->m2->forces += s->k * (s->m1->position - s->m2->position).unit() * ((s->m1->position - s->m2->position).norm() - s->rest_length);
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                //m->forces += gravity;
                //Vector2D acceleration = m->forces / m->mass;
                
                //for explicit Euler
                //Vector2D new_v = m->velocity + acceleration * delta_t;
                //Vector2D new_x = m->position + m->velocity * delta_t;

                //for semi-implicit Euler
                //Vector2D new_v = m->velocity + acceleration * delta_t;
                //Vector2D new_x = m->position + new_v * delta_t;

                //m->velocity = new_v;
                //m->position = new_x;



                // TODO (Part 2): Add global damping
                double damping_factor = 0.005;
                m->forces += gravity - damping_factor * m->velocity;
                Vector2D acceleration = m->forces / m->mass;

                //for explicit Euler
                //Vector2D new_v = m->velocity + acceleration * delta_t;
                //Vector2D new_x = m->position + m->velocity * delta_t;

                //for semi-implicit Euler
                Vector2D new_v = m->velocity + acceleration * delta_t;
                Vector2D new_x = m->position + new_v * delta_t;

                m->velocity = new_v;
                m->position = new_x;
            }
            

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            float distance = ((s->m1->position - s->m2->position).norm() - s->rest_length) / 2;
            if(s->m1->pinned && !s->m2->pinned)
            {
                s->m2->position += (s->m1->position - s->m2->position).unit() * distance * 2;
            }
            else if(!s->m1->pinned && s->m2->pinned)
            {
                s->m1->position += (s->m2->position - s->m1->position).unit() * distance * 2;
            }
            else if(!s->m1->pinned && !s->m2->pinned)
            {
                s->m1->position += (s->m2->position - s->m1->position).unit() * distance;
                s->m2->position += (s->m1->position - s->m2->position).unit() * distance;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                //Vector2D acceleration = gravity / m->mass;

                //for explicit Verlet
                //Vector2D new_x = m->position + (m->position - m->last_position) + acceleration * delta_t * delta_t;
              
                //m->position = new_x;
                //m->last_position = temp_position;


                // TODO (Part 4): Add global Verlet damping
                double damping_factor = 0.00005;
                Vector2D acceleration = gravity / m->mass;
                
                //for explicit Verlet
                Vector2D new_x = m->position + (1 - damping_factor) * (m->position - m->last_position) + acceleration * delta_t * delta_t;
    
                m->position = new_x;
                m->last_position = temp_position;
            }

        }
    }
}
