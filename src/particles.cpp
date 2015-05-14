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

class wtfemitter : public io::IAttributeExchangingObject {
public:
	virtual s32 emitt(u32 now, u32 timeSinceLastCall, scene::SParticle*& outArray) = 0;

};

class FixNumEmitter :  public wtfemitter {
private:
        core::array<irr::scene::SParticle> particles;

        u32 number;
        v3f pos;
        u32 emitted;
public:

        FixNumEmitter(int number) : number(number), emitted(0) {}

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
};

class PointEmitter : public wtfemitter {
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
					m_env, event->add_particlespawner.collisiondetection);

		ps->setID((s32) event->add_particlespawner.id);

		float pps = event->add_particlespawner.amount;
		float time = event->add_particlespawner.spawntime;

		if (time > 0)
			pps = pps / time;

		float minsize = 1;//event->add_particlespawner.minsize;
		float maxsize = 1;//event->add_particlespawner.maxsize;

		wtfemitter * em = new PointEmitter(
					random_v3f(v3f(-0.25,-0.25,-0.25), v3f(0.25, 0.25, 0.25))/10,
					3, //pps, // 5, //minpps
					4, //pps*2, // 20,//maxpps
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

		ps->setPosition(pos * BS/*  - intToFloat(m_env->getCameraOffset(), BS)*/);

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

		if (time != 0) {
			scene::ISceneNodeAnimator* pan =  m_smgr->createDeleteAnimator(time * 1000);
			ps->addAnimator(pan);
			pan->drop();
		}

		scene::ISceneNodeAnimator* wub = m_smgr->createFlyCircleAnimator(pos*BS, 300);
		ps->addAnimator(wub);
		wub->drop();

		return;


	}

	if (event->type == CE_SPAWN_PARTICLE) {
		//TODO: use FixNumEmitter here
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

	v3f particlepos = intToFloat(pos, BS)/* - intToFloat(m_env->getCameraOffset(), BS)*/;

	scene::CParticleSystemSceneNode2 *ps = new scene::CParticleSystemSceneNode2(
				m_smgr->getRootSceneNode(), m_smgr, -1,
				m_env, true);

	wtfemitter* em;
	ps->setDebugDataVisible(irr::scene::EDS_BBOX);


	em = new FixNumEmitter(number);
	ps->setEmitter(em);
	em->drop();

	ps->setPosition(particlepos);

	scene::IParticleAffector* paf1 = new MTGravityAffector(v3f(0.0, -0.1, 0.0), 2000);
	ps->addAffector(paf1);
	paf1->drop();

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

#include "ISceneManager.h"
#include "ICameraSceneNode.h"
#include "IVideoDriver.h"

#include "SViewFrustum.h"
namespace irr
{
namespace scene
{
//! constructor
CParticleSystemSceneNode2::CParticleSystemSceneNode2(ISceneNode* parent, ISceneManager* mgr, s32 id, ClientEnvironment *env, bool collision_detection)
	: ISceneNode(parent, mgr, id),
	  Emitter(0),
	  ParticleSize(core::dimension2d<f32>(5.0f, 5.0f)),
	  LastEmitTime(0),
	  Buffer(0),
	  ParticlesAreGlobal(true),
	  m_env(env),
	  m_gamedef(env->getGameDef()),
	  m_camera_offset(v3s16(0,0,0)),
	  collision_detection(collision_detection)
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
void CParticleSystemSceneNode2::setPosition(const core::vector3df newpos)
{
		RelativeTranslation = newpos;
}
//! Gets the particle emitter, which creates the particles.
wtfemitter* CParticleSystemSceneNode2::getEmitter()
{
	return Emitter;
}
//! Sets the particle emitter, which creates the particles.
void CParticleSystemSceneNode2::setEmitter(wtfemitter *emitter)
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
	doParticleSystem(getTimeMs());
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

	// adjust for mt/irrlicht camera offset
//	v3s16 camera_offset = m_env->getCameraOffset();
	v3f offset = intToFloat(m_camera_offset, BS);

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
		Buffer->Vertices[0+idx].Pos = particle.pos + horizontal + vertical - offset;
		Buffer->Vertices[0+idx].Color = particle.color;
		Buffer->Vertices[0+idx].Normal = view;
		Buffer->Vertices[1+idx].Pos = particle.pos + horizontal - vertical - offset;
		Buffer->Vertices[1+idx].Color = particle.color;
		Buffer->Vertices[1+idx].Normal = view;
		Buffer->Vertices[2+idx].Pos = particle.pos - horizontal - vertical - offset;
		Buffer->Vertices[2+idx].Color = particle.color;
		Buffer->Vertices[2+idx].Normal = view;
		Buffer->Vertices[3+idx].Pos = particle.pos - horizontal + vertical - offset;
		Buffer->Vertices[3+idx].Color = particle.color;
		Buffer->Vertices[3+idx].Normal = view;

//		for (int j=0; j<4; j++)
//			Buffer->Vertices[j+idx].Pos -= offset;

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
		return;
	}
	u32 now = time;
	u32 timediff = time - LastEmitTime;
	LastEmitTime = time;

	v3s16 offset = m_env->getCameraOffset();
	if (offset != m_camera_offset)
		m_camera_offset = offset;

	// run emitter
	if (Emitter && IsVisible)
	{
		SParticle* array = 0;
		s32 newParticles = Emitter->emitt(now, timediff, array);
		if (newParticles && array)
		{
			s32 j=Particles.size();
			if (newParticles > 16250-j)
				newParticles=16250-j;
			Particles.set_used(j+newParticles);
			for (s32 i=j; i<j+newParticles; ++i)
			{
				Particles[i]=array[i-j];

				AbsoluteTransformation.rotateVect(Particles[i].startVector);
				if (ParticlesAreGlobal)
					AbsoluteTransformation.transformVect(Particles[i].pos);
			}
		}
	}

	// run affectors
	core::list<IParticleAffector*>::Iterator ait = AffectorList.begin();
	for (; ait != AffectorList.end(); ++ait)
		(*ait)->affect(now, Particles.pointer(), Particles.size());
	if (ParticlesAreGlobal)
		Buffer->BoundingBox.reset(AbsoluteTransformation.getTranslation());
	else
		Buffer->BoundingBox.reset(core::vector3df(0,0,0));
	// animate all particles
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
			if (collision_detection) {
				v3f acc = v3f(0,0,0);
				irr::scene::SParticle p = Particles[i];
				float size = p.size.Width;
				core::aabbox3d<f32> box = core::aabbox3d<f32>
						(-size/2,-size/2,-size/2,size/2,size/2,size/2);

				// wut
				v3f pdir = Particles[i].vector*100;
				v3f ppos = Particles[i].pos;
				collisionMoveSimple(m_env, m_gamedef,
						    BS * 0.5, box,
						    0, timediff,
						    //ppos,
						    Particles[i].pos,
						    pdir,
//						    Particles[i].vector,
						    acc);
				//Particles[i].vector = pdir;
				//Particles[i].pos = ppos;
			} else {
				Particles[i].pos += (Particles[i].vector * scale);
			}

			Buffer->BoundingBox.addInternalPoint(Particles[i].pos);
			++i;
		}
	}
	const f32 m = (ParticleSize.Width > ParticleSize.Height ? ParticleSize.Width : ParticleSize.Height) * 0.5f;
	Buffer->BoundingBox.MaxEdge.X += m;
	Buffer->BoundingBox.MaxEdge.Y += m;
	Buffer->BoundingBox.MaxEdge.Z += m;
	Buffer->BoundingBox.MinEdge.X -= m;
	Buffer->BoundingBox.MinEdge.Y -= m;
	Buffer->BoundingBox.MinEdge.Z -= m;

	v3f camera_offset = intToFloat(m_camera_offset, BS);
	Buffer->BoundingBox.MaxEdge -= camera_offset;
	Buffer->BoundingBox.MinEdge -= camera_offset;
	if (ParticlesAreGlobal)
	{
		core::matrix4 absinv( AbsoluteTransformation, core::matrix4::EM4CONST_INVERSE );
		absinv.transformBoxEx(Buffer->BoundingBox);
	}
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
