using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ParticleSubFollowMe : MonoBehaviour
{
    ParticleSystem m_System;
    public ParticleSystem subEmitter;
    ParticleSystem.Particle[] m_Particles;
    ParticleSystem.Particle[] m_ParticlesSub;

    void OnEnable()
    {
        m_System = GetComponent<ParticleSystem>();
        m_Particles = new ParticleSystem.Particle[m_System.main.maxParticles];
        m_ParticlesSub = new ParticleSystem.Particle[subEmitter.main.maxParticles];
    }

    private void LateUpdate()
    {
        int numParticlesAlive = m_System.GetParticles(m_Particles);
        int numParticlesAliveSub = subEmitter.GetParticles(m_ParticlesSub);

        for (int i = 0; i < numParticlesAlive; i++)
        {
            for (int j = 0; j < numParticlesAliveSub; j++)
            {
                //m_ParticlesSub[j].position += m_Particles[i].position;
                m_ParticlesSub[j].position += m_Particles[i].position + m_Particles[i].startSize3D.y * m_Particles[i].totalVelocity.normalized;
                //Debug.Log(m_Particles[i].velocity);   
            }
        }
        subEmitter.SetParticles(m_ParticlesSub, numParticlesAliveSub);
    }
}