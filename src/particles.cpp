/*
Minetest
Copyright (C) 2013 celeron55, Perttu Ahola <celeron55@gmail.com>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation; either version 2.1 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include "particles.h"
#include "constants.h"
#include "debug.h"
#include "main.h" // For g_profiler and g_settings
#include "settings.h"
#include "client/tile.h"
#include "gamedef.h"
#include "collision.h"
#include <stdlib.h>
#include "util/numeric.h"
#include "light.h"
#include "environment.h"
#include "clientmap.h"
#include "mapnode.h"
#include "client.h"

#include "nodedef.h"
class Map;
class IGameDef;
class Environment;
#define PP(x) "("<<(x).X<<","<<(x).Y<<","<<(x).Z<<")"
/*

	Utility
*/

v3f random_v3f(v3f min, v3f max)
{
	return v3f( rand()/(float)RAND_MAX*(max.X-min.X)+min.X,
			rand()/(float)RAND_MAX*(max.Y-min.Y)+min.Y,
			rand()/(float)RAND_MAX*(max.Z-min.Z)+min.Z);
}

class wtfEmitter : public irr::scene::IParticleEmitter {
public:
        virtual void setDirection( const core::vector3df& newDirection ) {  }
        virtual void setMinParticlesPerSecond( u32 minPPS ) {  }
        virtual void setMaxParticlesPerSecond( u32 maxPPS ) {  }
        virtual void setMinStartColor( const video::SColor& color ) {  }
        virtual void setMaxStartColor( const video::SColor& color ) {  }
        virtual void setMaxLifeTime( const u32 t ) {  }
        virtual void setMinLifeTime( const u32 t ) { }
        virtual u32 getMaxLifeTime() const { return 1; }
        virtual u32 getMinLifeTime() const { return 1; }
        virtual void setMaxAngleDegrees(const s32 t ) {  }
        virtual s32 getMaxAngleDegrees() const { return 0; }
        virtual void setMaxStartSize( const core::dimension2df& size ) { }
        virtual void setMinStartSize( const core::dimension2df& size ) {  }
        virtual void setCenter( const core::vector3df& center ) {  }
        virtual void setRadius( f32 radius ) {  }
        virtual const core::vector3df& getDirection() const { return v3f(0.0, 0.0, 0.0); }
        virtual u32 getMinParticlesPerSecond() const { return 1; }
        virtual u32 getMaxParticlesPerSecond() const { return 1; }
        virtual const video::SColor& getMinStartColor() const { return video::SColor(1); }
        virtual const video::SColor& getMaxStartColor() const { return video::SColor(1);; }
        virtual const core::dimension2df& getMaxStartSize() const { return core::dimension2d<f32>(0.2, 0.2); }
        virtual const core::dimension2df& getMinStartSize() const { return core::dimension2d<f32>(0.2, 0.2); }
        virtual const core::vector3df& getCenter() const { return v3f(0.0, 0.0, 0.0); }
        virtual f32 getRadius() const { return 1.; }
};

class FixNumEmitter :  public wtfEmitter {
private:
        core::array<irr::scene::SParticle> particles;

        u32 number;
        v3f pos;
        u32 emitted;
public:

        FixNumEmitter(/*v3f pos,*/ int number) : number(number), pos(pos), emitted(0) {}

        s32 emitt(u32 now, u32 timeSinceLastCall, irr::scene::SParticle*& outArray)
        {
                if (emitted > 0) return 0;

                particles.set_used(0);

                irr::scene::SParticle p;
                for(u32 i=0; i<number; ++i)
                {
                        v3f particlepos = v3f(
                                                rand() %100 /200. - 0.25,
                                                rand() %100 /200. - 0.25,
                                                rand() %100 /200. - 0.25);

                        p.pos.set(particlepos);

                        v3f velocity((rand() % 100 / 50. - 1) / 1.5,
                                      rand() % 100 / 35.,
                                     (rand() % 100 / 50. - 1) / 1.5);
                        p.vector = velocity/100;
                        p.startVector = p.vector;

                        p.startTime = now;
                        p.endTime = now + 500 + (rand() % (2000 - 500));;

                        p.color = video::SColor(255.0, 255.0, 255.0, 255.0);
                        p.startColor = p.color;
                        float size = rand() % 64 / 51.2;
                        p.startSize = core::dimension2d<f32>(size, size);
                        p.size = p.startSize;
                        particles.push_back(p);
                }
                outArray = particles.pointer();
                emitted += particles.size();

                return particles.size();
        }

        virtual irr::scene::E_PARTICLE_EMITTER_TYPE getType() const {
                return (irr::scene::E_PARTICLE_EMITTER_TYPE) (irr::scene::EPET_COUNT+1);
        }

        void restart() { emitted=0; }

};

class PointEmitter : public wtfEmitter {
public:
        PointEmitter(const core::vector3df& direction,
                   u32 minParticlesPerSecond, u32 maxParticlesPerSecond,
                   video::SColor minStartColor, video::SColor maxStartColor,
                   u32 lifeTimeMin, u32 lifeTimeMax, s32 maxAngleDegrees,
                   const core::dimension2df& minStartSize, const core::dimension2df& maxStartSize)
                : Direction(direction),
                  MaxStartSize(maxStartSize), MinStartSize(minStartSize),
                  MinParticlesPerSecond(minParticlesPerSecond),
                  MaxParticlesPerSecond(maxParticlesPerSecond),
                  MinStartColor(minStartColor), MaxStartColor(maxStartColor),
                  MinLifeTime(lifeTimeMin), MaxLifeTime(lifeTimeMax),
                  Time(0), MaxAngleDegrees(maxAngleDegrees)
        {
#ifdef _DEBUG
                setDebugName("CParticlePointEmitter");
#endif
        }
        //! Prepares an array with new particles to emitt into the system
        //! and returns how much new particles there are.
        s32 emitt(u32 now, u32 timeSinceLastCall, irr::scene::SParticle*& outArray)
        {
                Time += timeSinceLastCall;
                const u32 pps = (MaxParticlesPerSecond - MinParticlesPerSecond);
                const f32 perSecond = pps ? ((f32)MinParticlesPerSecond + rand() * pps) : MinParticlesPerSecond;
                const f32 everyWhatMillisecond = 1000.0f / perSecond;
                if (Time > everyWhatMillisecond)
                {
                        Time = 0;
                        Particle.startTime = now;
                        Particle.vector = Direction;
                        if (MaxAngleDegrees)
                        {
                                core::vector3df tgt = Direction;
                                tgt.rotateXYBy(rand() * MaxAngleDegrees);
                                tgt.rotateYZBy(rand() * MaxAngleDegrees);
                                tgt.rotateXZBy(rand() * MaxAngleDegrees);
                                Particle.vector = tgt;
                        }
                        Particle.endTime = now + MinLifeTime;
                        if (MaxLifeTime != MinLifeTime)
                                Particle.endTime += rand() % (MaxLifeTime - MinLifeTime);
                        if (MinStartColor==MaxStartColor)
                                Particle.color=MinStartColor;
                        else
                                Particle.color = MinStartColor.getInterpolated(MaxStartColor, rand());
                        Particle.startColor = Particle.color;
                        Particle.startVector = Particle.vector;
                        if (MinStartSize==MaxStartSize)
                                Particle.startSize = MinStartSize;
                        else
                                Particle.startSize = MinStartSize.getInterpolated(MaxStartSize, rand());
                        Particle.size = Particle.startSize;
                        outArray = &Particle;
                        return 1;
                }
                return 0;
        }


private:
        irr::scene::SParticle Particle;
        core::array<irr::scene::SParticle> Particles;
        core::vector3df Direction;
        core::dimension2df MaxStartSize, MinStartSize;
        u32 MinParticlesPerSecond, MaxParticlesPerSecond;
        video::SColor MinStartColor, MaxStartColor;
        u32 MinLifeTime, MaxLifeTime;
        u32 Time;
        s32 MaxAngleDegrees;
};



class CollisionAffector : public irr::scene::IParticleAffector
{
public:
        CollisionAffector(ClientEnvironment &env)
        {
                m_env = &env;
                m_gamedef = m_env->getGameDef();
        }
        void affect(u32 now, irr::scene::SParticle* particlearray, u32 count)
        {
                if( LastTime == 0 )
                {
                        LastTime = now;
                        return;
                }
                f32 timeDelta = ( now - LastTime ) / 1000.0f;

                LastTime = now;
                if( !Enabled )
                        return;

                v3f acc = v3f(0.0, 0.0, 0.0);
                for(u32 i=0; i<count; ++i)
                {
                        irr::scene::SParticle p = particlearray[i];
                        float size = p.size.Width;
                        core::aabbox3d<f32> box = core::aabbox3d<f32>
                                        (-size/2,-size/2,-size/2,size/2,size/2,size/2);

                        //to ensure particles collide with correct position
                        v3f off = intToFloat(m_env->getCameraOffset(), BS);

                        particlearray[i].pos += off;

                        collisionMoveSimple(m_env, m_gamedef,
                                            BS * 0.5, box,
                                            0, timeDelta,
                                            particlearray[i].pos,
                                            particlearray[i].vector,
                                            acc);

                        particlearray[i].pos -= off;

                        // TODO: create light affector?
//                        try{
//                                v3s16 p = v3s16(
//                                                        floor(pos.X+0.5),
//                                                        floor(pos.Y+0.5),
//                                                        floor(pos.Z+0.5)
//                                                        );
//                                MapNode n = m_env->getClientMap().getNode(p);
//                                light = n.getLightBlend(m_env->getDayNightRatio(), m_gamedef->ndef());
//                        }
//                        catch(InvalidPositionException &e){
//                                light = blend_light(m_env->getDayNightRatio(), LIGHT_SUN, 0);
//                        }
//                        u8 plight = decode_light(light);
//                        particlearray[i].color = video::SColor(255, plight, plight, plight);
                }
        }
        virtual irr::scene::E_PARTICLE_AFFECTOR_TYPE getType() const {
                return (irr::scene::E_PARTICLE_AFFECTOR_TYPE) (irr::scene::EPAT_COUNT+1);
        }
        irr::u32 LastTime;
        IGameDef *m_gamedef;
        ClientEnvironment *m_env;
};

class MTGravityAffector : public irr::scene::IParticleAffector
{
public:
        MTGravityAffector(const core::vector3df& gravity, u32 timeForceLost)
                : TimeForceLost(static_cast<f32>(timeForceLost)), Gravity(gravity)
        {
#ifdef _DEBUG
                setDebugName("CParticleGravityAffector");
#endif
        }
//! Affects an array of particles.
        void affect(u32 now, irr::scene::SParticle* particlearray, u32 count)
        {
                if (!Enabled)
                        return;
                f32 d;
                for (u32 i=0; i<count; ++i)
                {
                        d = (now - particlearray[i].startTime) / TimeForceLost;
                        if (d > 1.0f)
                                d = 1.0f;
                        if (d < 0.0f)
                                d = 0.0f;
                        d = 1.0f - d;
                        particlearray[i].vector = particlearray[i].startVector.getInterpolated(Gravity, d);
                }
        }
        virtual irr::scene::E_PARTICLE_AFFECTOR_TYPE getType() const {
                return (irr::scene::E_PARTICLE_AFFECTOR_TYPE) (irr::scene::EPAT_COUNT+4);
        }
private:
        f32 TimeForceLost;
        v3f Gravity;
};

ParticleManager::ParticleManager(ClientEnvironment* env, irr::scene::ISceneManager* smgr) :
	m_env(env),
	m_smgr(smgr)
{}

ParticleManager::~ParticleManager()
{
	clearAll();
}

void ParticleManager::clearAll ()
{
	return;
}

void ParticleManager::handleParticleEvent(ClientEvent *event, IGameDef *gamedef, LocalPlayer *player)
{
	if (event->type == CE_DELETE_PARTICLESPAWNER) {
		scene::ISceneNode *node = m_smgr->getSceneNodeFromId(
					event->delete_particlespawner.id);
		if (node)
			m_smgr->addToDeletionQueue(node);
		return;
	}

	if (event->type == CE_ADD_PARTICLESPAWNER) {

		scene::ISceneNode *node = m_smgr->getSceneNodeFromId(
					event->delete_particlespawner.id);
		if (node)
			m_smgr->addToDeletionQueue(node);

		video::ITexture *texture =
				gamedef->tsrc()->getTextureForMesh(*(event->add_particlespawner.texture));

		v3f pos = *event->add_particlespawner.minpos;


		scene::CParticleSystemSceneNode2 *ps = new scene::CParticleSystemSceneNode2(
					m_smgr->getRootSceneNode(), m_smgr, -1,
					m_env, pos*BS);

		ps->setID((s32) event->add_particlespawner.id);

		float pps = event->add_particlespawner.amount;
		float time = event->add_particlespawner.spawntime;

		if (time > 0)
			pps = pps / time;

		float minsize = 1;//event->add_particlespawner.minsize;
		float maxsize = 1;//event->add_particlespawner.maxsize;

		wtfEmitter * em = new PointEmitter(
					random_v3f(v3f(-0.25,-0.25,-0.25), v3f(0.25, 0.25, 0.25))/10,
					4, // 5, //minpps
					12, // 20,//maxpps
					video::SColor(255.0, 255.0, 255.0, 255.0), //mincol,
					video::SColor(255.0, 255.0, 255.0, 255.0), //maxcol,
					event->add_particlespawner.minexptime*1000,
					event->add_particlespawner.maxexptime*1000,
					360, //maxdeg,
					core::dimension2d<f32>(minsize, minsize),
					core::dimension2d<f32>(maxsize, maxsize));

		ps->setEmitter(em);
		em->drop();

		ps->setMaterialTexture(0, texture);
		ps->setAutomaticCulling(scene::EAC_OFF);
		ps->setDebugDataVisible(irr::scene::EDS_BBOX);

		ps->setPosition(pos * BS);

		ps->setMaterialFlag(video::EMF_LIGHTING, false);
		ps->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
		ps->setMaterialFlag(video::EMF_BILINEAR_FILTER, false);
		ps->setMaterialFlag(video::EMF_FOG_ENABLE, true);
		ps->setMaterialType(video::EMT_TRANSPARENT_ALPHA_CHANNEL);

		// TODO: create acceleration affector similar to GravityAffector
		// with min/max acceleration
		scene::IParticleAffector* paf1 = new MTGravityAffector(v3f(0.0, 0.01, 0.0), 2000);
		ps->addAffector(paf1);
		paf1->drop();

		if (event->add_particlespawner.collisiondetection) {
			scene::IParticleAffector* paf2 = new CollisionAffector(*m_env);
			ps->addAffector(paf2);
			paf2->drop();
		}

		if (time != 0) {
			scene::ISceneNodeAnimator* pan =  m_smgr->createDeleteAnimator(time * 1000);
			ps->addAnimator(pan);
			pan->drop();
		}
		return;
	}

	if (event->type == CE_SPAWN_PARTICLE) {
		//TODO: use FixNumEmitter here

		//		video::ITexture *texture =
		//			gamedef->tsrc()->getTextureForMesh(*(event->spawn_particle.texture));

		//		Particle* toadd = new Particle(gamedef, smgr, player, m_env,
		//				*event->spawn_particle.pos,
		//				*event->spawn_particle.vel,
		//				*event->spawn_particle.acc,
		//				event->spawn_particle.expirationtime,
		//				event->spawn_particle.size,
		//				event->spawn_particle.collisiondetection,
		//				event->spawn_particle.vertical,
		//				texture,
		//				v2f(0.0, 0.0),
		//				v2f(1.0, 1.0));

		//		addParticle(toadd);
		return;
	}
}

void ParticleManager::addDiggingParticles(IGameDef* gamedef,
		LocalPlayer *player, v3s16 pos, const TileSpec tiles[])
{
		addNodeParticle(gamedef, player, pos, tiles, 32);
}

void ParticleManager::addPunchingParticles(IGameDef* gamedef,
		LocalPlayer *player, v3s16 pos, const TileSpec tiles[])
{
	addNodeParticle(gamedef, player, pos, tiles, 1);
}

void ParticleManager::addNodeParticle(IGameDef* gamedef, LocalPlayer *player,
				      v3s16 pos, const TileSpec tiles[], int number)
{
	// Texture
	u8 texid = myrand_range(0, 5);
	video::ITexture *texture = tiles[texid].texture;

	v3f particlepos = intToFloat(pos, BS);

	scene::CParticleSystemSceneNode2 *ps = new scene::CParticleSystemSceneNode2(
				m_smgr->getRootSceneNode(), m_smgr, -1,
				m_env, particlepos);
	scene::IParticleEmitter* em;
	ps->setDebugDataVisible(irr::scene::EDS_BBOX);

	ps->setPosition(particlepos);

	em = new FixNumEmitter(number);
	ps->setEmitter(em);
	em->drop();

	scene::IParticleAffector* paf1 = new MTGravityAffector(v3f(0.0, -0.1, 0.0), 2000);
	ps->addAffector(paf1);
	paf1->drop();

	scene::IParticleAffector* paf2 = new CollisionAffector(*m_env);
	ps->addAffector(paf2);
	paf2->drop();

	scene::ISceneNodeAnimator* pan =  m_smgr->createDeleteAnimator(2000); //delete after max lifetime
	ps->addAnimator(pan);
	pan->drop();

	ps->setMaterialTexture(0, texture);

	ps->setMaterialFlag(video::EMF_LIGHTING, false);
	ps->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
	ps->setMaterialFlag(video::EMF_BILINEAR_FILTER, false);
	ps->setMaterialFlag(video::EMF_FOG_ENABLE, true);
	ps->setMaterialType(video::EMT_TRANSPARENT_ALPHA_CHANNEL);
}
















// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h
//#include "os.h"
#include "ISceneManager.h"
#include "ICameraSceneNode.h"
#include "IVideoDriver.h"

#include "SViewFrustum.h"
namespace irr
{
namespace scene
{

//! Bitflags to control particle behavior
enum EParticleBehavior
{
	//! Continue emitting new particles even when the node is invisible
	EPB_INVISIBLE_EMITTING = 1,

	//! Continue affecting particles even when the node is invisible
	EPB_INVISIBLE_AFFECTING = 2,

	//! Continue updating particle positions or deleting them even when the node is invisible
	EPB_INVISIBLE_ANIMATING = 4,

	//! Clear all particles when node gets invisible
	EPB_CLEAR_ON_INVISIBLE = 8,

	//! Particle movement direction on emitting ignores the node rotation
	//! This is mainly to allow backward compatible behavior to Irrlicht 1.8
	EPB_EMITTER_VECTOR_IGNORE_ROTATION = 16,

	//! On emitting global particles interpolate the positions randomly between the last and current node transformations.
	//! This can be set to avoid gaps caused by fast node movement or low framerates, but will be somewhat
	//! slower to calculate.
	EPB_EMITTER_FRAME_INTERPOLATION = 32
};
//! constructor
CParticleSystemSceneNode2::CParticleSystemSceneNode2(ISceneNode* parent, ISceneManager* mgr, s32 id, ClientEnvironment *env
						   /*const core::vector3df& position, const core::vector3df& rotation,
						   const core::vector3df& scale*/, v3f rpos)
	: ISceneNode(parent, mgr, id/*, position, rotation, scale*/),
	  Emitter(0), ParticleSize(core::dimension2d<f32>(5.0f, 5.0f)), LastEmitTime(0),
	  Buffer(0), ParticlesAreGlobal(true), m_env(env), rpos(rpos), m_camera_offset(m_env->getCameraOffset()), once(true)
{
#ifdef _DEBUG
	setDebugName("CParticleSystemSceneNode2");
#endif
	Buffer = new SMeshBuffer();
}
//! destructor
CParticleSystemSceneNode2::~CParticleSystemSceneNode2()
{
	if (Emitter)
		Emitter->drop();
	if (Buffer)
		Buffer->drop();
	removeAllAffectors();
}
//! Gets the particle emitter, which creates the particles.
IParticleEmitter* CParticleSystemSceneNode2::getEmitter()
{
	return Emitter;
}
//! Sets the particle emitter, which creates the particles.
void CParticleSystemSceneNode2::setEmitter(IParticleEmitter *emitter)
{
	if (emitter == Emitter)
		return;
	if (Emitter)
		Emitter->drop();
	Emitter = emitter;
	if (Emitter)
		Emitter->grab();
}
//! Adds new particle effector to the particle system.
void CParticleSystemSceneNode2::addAffector(IParticleAffector* affector)
{
	affector->grab();
	AffectorList.push_back(affector);
}
//! Get a list of all particle affectors.
const core::list<IParticleAffector*>& CParticleSystemSceneNode2::getAffectors() const
{
	return AffectorList;
}
//! Removes all particle affectors in the particle system.
void CParticleSystemSceneNode2::removeAllAffectors()
{
	core::list<IParticleAffector*>::Iterator it = AffectorList.begin();
	while (it != AffectorList.end())
	{
		(*it)->drop();
		it = AffectorList.erase(it);
	}
}
//! Returns the material based on the zero based index i.
video::SMaterial& CParticleSystemSceneNode2::getMaterial(u32 i)
{
	return Buffer->Material;
}
//! Returns amount of materials used by this scene node.
u32 CParticleSystemSceneNode2::getMaterialCount() const
{
	return 1;
}
//! pre render event
void CParticleSystemSceneNode2::OnRegisterSceneNode()
{
	doParticleSystem(getTimeMs());//os::Timer::getTime());
	if (IsVisible && (Particles.size() != 0))
	{
		SceneManager->registerNodeForRendering(this);
		ISceneNode::OnRegisterSceneNode();
	}
}
//! render
void CParticleSystemSceneNode2::render()
{
	video::IVideoDriver* driver = SceneManager->getVideoDriver();
	ICameraSceneNode* camera = SceneManager->getActiveCamera();
	if (!camera || !driver)
		return;
#if 0
	// calculate vectors for letting particles look to camera
	core::vector3df view(camera->getTarget() - camera->getAbsolutePosition());
	view.normalize();
	view *= -1.0f;
#else
	const core::matrix4 &m = camera->getViewFrustum()->getTransform( video::ETS_VIEW );
	const core::vector3df view ( -m[2], -m[6] , -m[10] );
#endif
	// reallocate arrays, if they are too small
	reallocateBuffers();

	bool update = false;
	v3s16 camera_offset = m_env->getCameraOffset();
	if (camera_offset != m_camera_offset) {
		update = true;
	}

	v3f offset = intToFloat(camera_offset, BS);

	if (once) {
		once = false;
		RelativeTranslation = rpos - intToFloat(m_camera_offset, BS);
		for (u32 i=0; i<Particles.size(); ++i)
		{
			//Particles[i].pos += intToFloat(m_camera_offset, BS);
			Particles[i].pos -= intToFloat(m_camera_offset, BS);
		}
	}


	if (update) {
		once = false;
		std::cout << "offset : " << PP(offset) << std::endl;
		RelativeTranslation = rpos - offset;

		for (u32 i=0; i<Particles.size(); ++i)
		{
			Particles[i].pos += intToFloat(m_camera_offset, BS);
			Particles[i].pos -= intToFloat(camera_offset, BS);
		}
		m_camera_offset = camera_offset;
	}

	// create particle vertex data
	s32 idx = 0;
	for (u32 i=0; i<Particles.size(); ++i)
	{
		const SParticle& particle = Particles[i];
#if 0
		core::vector3df horizontal = camera->getUpVector().crossProduct(view);
		horizontal.normalize();
		horizontal *= 0.5f * particle.size.Width;
		core::vector3df vertical = horizontal.crossProduct(view);
		vertical.normalize();
		vertical *= 0.5f * particle.size.Height;
#else
		f32 f;
		f = 0.5f * particle.size.Width;
		const core::vector3df horizontal ( m[0] * f, m[4] * f, m[8] * f );
		f = -0.5f * particle.size.Height;
		const core::vector3df vertical ( m[1] * f, m[5] * f, m[9] * f );
#endif
		Buffer->Vertices[0+idx].Pos = particle.pos + horizontal + vertical;
		Buffer->Vertices[0+idx].Color = particle.color;
		Buffer->Vertices[0+idx].Normal = view;
		Buffer->Vertices[1+idx].Pos = particle.pos + horizontal - vertical;
		Buffer->Vertices[1+idx].Color = particle.color;
		Buffer->Vertices[1+idx].Normal = view;
		Buffer->Vertices[2+idx].Pos = particle.pos - horizontal - vertical;
		Buffer->Vertices[2+idx].Color = particle.color;
		Buffer->Vertices[2+idx].Normal = view;
		Buffer->Vertices[3+idx].Pos = particle.pos - horizontal + vertical;
		Buffer->Vertices[3+idx].Color = particle.color;
		Buffer->Vertices[3+idx].Normal = view;

//			Buffer->Vertices[0+idx].Pos -= offset;
//			Buffer->Vertices[1+idx].Pos -= offset;
//			Buffer->Vertices[2+idx].Pos -= offset;
//			Buffer->Vertices[3+idx].Pos -= offset;

		idx +=4;
	}
	// render all
	core::matrix4 mat;
	if (!ParticlesAreGlobal)
		mat.setTranslation(AbsoluteTransformation.getTranslation());
	driver->setTransform(video::ETS_WORLD, mat);
	driver->setMaterial(Buffer->Material);
	driver->drawVertexPrimitiveList(Buffer->getVertices(), Particles.size()*4,
					Buffer->getIndices(), Particles.size()*2, video::EVT_STANDARD, EPT_TRIANGLES,Buffer->getIndexType());

	// for debug purposes only:
	if ( DebugDataVisible & scene::EDS_BBOX )
	{
		driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);
		video::SMaterial deb_m;
		deb_m.Lighting = false;
		driver->setMaterial(deb_m);
		driver->draw3DBox(Buffer->BoundingBox, video::SColor(0,255,255,255));
	}
}
//! returns the axis aligned bounding box of this node
const core::aabbox3d<f32>& CParticleSystemSceneNode2::getBoundingBox() const
{
	return Buffer->getBoundingBox();
}
void CParticleSystemSceneNode2::doParticleSystem(u32 time)
{
	if (LastEmitTime==0)
	{
		LastEmitTime = time;
		LastAbsoluteTransformation = AbsoluteTransformation;
		return;
	}

	u32 now = time;
	u32 timediff = time - LastEmitTime;
	LastEmitTime = time;
	bool visible = isVisible();
	int behavior = 1;//getParticleBehavior();
	// run emitter
	if (Emitter && (visible || behavior & EPB_INVISIBLE_EMITTING) )
	{
		SParticle* array = 0;
		s32 newParticles = Emitter->emitt(now, timediff, array);
		if (newParticles && array)
		{
			s32 j=Particles.size();
			if (newParticles > 16250-j)	// avoid having more than 64k vertices in the scenenode
				newParticles=16250-j;
			Particles.set_used(j+newParticles);
			for (s32 i=j; i<j+newParticles; ++i)
			{
				Particles[i]=array[i-j];
				if ( ParticlesAreGlobal && behavior & EPB_EMITTER_FRAME_INTERPOLATION )
				{
					// Interpolate between current node transformations and last ones.
					// (Lazy solution - calculating twice and interpolating results)
					f32 randInterpolate = (f32)(rand() % 101) / 100.f;	// 0 to 1
					core::vector3df posNow(Particles[i].pos);
					core::vector3df posLast(Particles[i].pos);
					AbsoluteTransformation.transformVect(posNow);
					LastAbsoluteTransformation.transformVect(posLast);
					Particles[i].pos = posNow.getInterpolated(posLast, randInterpolate);
					if ( !(behavior & EPB_EMITTER_VECTOR_IGNORE_ROTATION) )
					{
						core::vector3df vecNow(Particles[i].startVector);
						core::vector3df vecOld(Particles[i].startVector);
						AbsoluteTransformation.rotateVect(vecNow);
						LastAbsoluteTransformation.rotateVect(vecOld);
						Particles[i].startVector = vecNow.getInterpolated(vecOld, randInterpolate);
						vecNow = Particles[i].vector;
						vecOld = Particles[i].vector;
						AbsoluteTransformation.rotateVect(vecNow);
						LastAbsoluteTransformation.rotateVect(vecOld);
						Particles[i].vector = vecNow.getInterpolated(vecOld, randInterpolate);
					}
				}
				else
				{
					if (ParticlesAreGlobal)
						AbsoluteTransformation.transformVect(Particles[i].pos);
					if ( !(behavior & EPB_EMITTER_VECTOR_IGNORE_ROTATION) )
					{
						if (!ParticlesAreGlobal)
							AbsoluteTransformation.rotateVect(Particles[i].pos);
						AbsoluteTransformation.rotateVect(Particles[i].startVector);
						AbsoluteTransformation.rotateVect(Particles[i].vector);
					}
				}
			}
		}
	}
	// run affectors
	if ( visible || behavior & EPB_INVISIBLE_AFFECTING )
	{
		core::list<IParticleAffector*>::Iterator ait = AffectorList.begin();
		for (; ait != AffectorList.end(); ++ait)
			(*ait)->affect(now, Particles.pointer(), Particles.size());
	}
	if (ParticlesAreGlobal)
		Buffer->BoundingBox.reset(AbsoluteTransformation.getTranslation());
	else
		Buffer->BoundingBox.reset(core::vector3df(0,0,0));
	// animate all particles
	if ( visible || behavior & EPB_INVISIBLE_ANIMATING )
	{
		f32 scale = (f32)timediff;
		for (u32 i=0; i<Particles.size();)
		{
			// erase is pretty expensive!
			if (now > Particles[i].endTime)
			{
				// Particle order does not seem to matter.
				// So we can delete by switching with last particle and deleting that one.
				// This is a lot faster and speed is very important here as the erase otherwise
				// can cause noticable freezes.
				Particles[i] = Particles[Particles.size()-1];
				Particles.erase( Particles.size()-1 );
			}
			else
			{
				Particles[i].pos += (Particles[i].vector * scale);
				Buffer->BoundingBox.addInternalPoint(Particles[i].pos);
				++i;
			}
		}
	}
	const f32 m = (ParticleSize.Width > ParticleSize.Height ? ParticleSize.Width : ParticleSize.Height) * 0.5f;
	Buffer->BoundingBox.MaxEdge.X += m;
	Buffer->BoundingBox.MaxEdge.Y += m;
	Buffer->BoundingBox.MaxEdge.Z += m;
	Buffer->BoundingBox.MinEdge.X -= m;
	Buffer->BoundingBox.MinEdge.Y -= m;
	Buffer->BoundingBox.MinEdge.Z -= m;
	if (ParticlesAreGlobal)
	{
		core::matrix4 absinv( AbsoluteTransformation, core::matrix4::EM4CONST_INVERSE );
		absinv.transformBoxEx(Buffer->BoundingBox);
	}
	LastAbsoluteTransformation = AbsoluteTransformation;
}
//! Sets if the particles should be global. If it is, the particles are affected by
//! the movement of the particle system scene node too, otherwise they completely
//! ignore it. Default is true.
void CParticleSystemSceneNode2::setParticlesAreGlobal(bool global)
{
	ParticlesAreGlobal = global;
}
//! Remove all currently visible particles
void CParticleSystemSceneNode2::clearParticles()
{
	Particles.set_used(0);
}
//! Sets if the node should be visible or not.
void CParticleSystemSceneNode2::setVisible(bool isVisible)
{
	ISceneNode::setVisible(isVisible);
	if ( !isVisible /*&& getParticleBehavior()*/ & EPB_CLEAR_ON_INVISIBLE )
	{
		clearParticles();
		LastEmitTime = 0;
	}
}
//! Sets the size of all particles.
void CParticleSystemSceneNode2::setParticleSize(const core::dimension2d<f32> &size)
{
	//os::Printer::log("setParticleSize is deprecated, use setMinStartSize/setMaxStartSize in emitter.", irr::ELL_WARNING);
	//A bit of a hack, but better here than in the particle code
	if (Emitter)
	{
		Emitter->setMinStartSize(size);
		Emitter->setMaxStartSize(size);
	}
	ParticleSize = size;
}
void CParticleSystemSceneNode2::reallocateBuffers()
{
	if (Particles.size() * 4 > Buffer->getVertexCount() ||
	    Particles.size() * 6 > Buffer->getIndexCount())
	{
		u32 oldSize = Buffer->getVertexCount();
		Buffer->Vertices.set_used(Particles.size() * 4);
		u32 i;
		// fill remaining vertices
		for (i=oldSize; i<Buffer->Vertices.size(); i+=4)
		{
			Buffer->Vertices[0+i].TCoords.set(0.0f, 0.0f);
			Buffer->Vertices[1+i].TCoords.set(0.0f, 1.0f);
			Buffer->Vertices[2+i].TCoords.set(1.0f, 1.0f);
			Buffer->Vertices[3+i].TCoords.set(1.0f, 0.0f);
		}
		// fill remaining indices
		u32 oldIdxSize = Buffer->getIndexCount();
		u32 oldvertices = oldSize;
		Buffer->Indices.set_used(Particles.size() * 6);
		for (i=oldIdxSize; i<Buffer->Indices.size(); i+=6)
		{
			Buffer->Indices[0+i] = (u16)0+oldvertices;
			Buffer->Indices[1+i] = (u16)2+oldvertices;
			Buffer->Indices[2+i] = (u16)1+oldvertices;
			Buffer->Indices[3+i] = (u16)0+oldvertices;
			Buffer->Indices[4+i] = (u16)3+oldvertices;
			Buffer->Indices[5+i] = (u16)2+oldvertices;
			oldvertices += 4;
		}
	}
}
} // end namespace scene
} // end namespace irr

