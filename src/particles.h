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

#ifndef PARTICLES_HEADER
#define PARTICLES_HEADER

#define DIGGING_PARTICLES_AMOUNT 10

#include <iostream>
#include "irrlichttypes_extrabloated.h"
#include "client/tile.h"
#include "localplayer.h"
#include "environment.h"

struct ClientEvent;
class ParticleManager;

/**
 * Class doing particle as well as their spawners handling
 */
class ParticleManager
{
public:
	ParticleManager(ClientEnvironment* env, irr::scene::ISceneManager* smgr);
	~ParticleManager();

	void handleParticleEvent(ClientEvent *event, IGameDef *gamedef, LocalPlayer *player);

	void addDiggingParticles(IGameDef* gamedef, LocalPlayer *player,
				 v3s16 pos, const TileSpec tiles[]);

	void addPunchingParticles(IGameDef* gamedef, LocalPlayer *player,
				  v3s16 pos, const TileSpec tiles[]);

	void addNodeParticle(IGameDef* gamedef, LocalPlayer *player,
			     v3s16 pos, const TileSpec tiles[], int number);

protected:

private:
//	void stepSpawners (float dtime);

	void clearAll ();

	//std::map<u32, s32> irrlicht_spawners; //mt id/irrlicht node id
	//v3s16 m_camera_offset;


	ClientEnvironment* m_env;
	irr::scene::ISceneManager* m_smgr;
	//JMutex m_spawner_list_lock;
};

#endif































































































// Copyright (C) 2002-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h
#ifndef __C_PARTICLE_SYSTEM_SCENE_NODE_H_INCLUDED__
#define __C_PARTICLE_SYSTEM_SCENE_NODE_H_INCLUDED__
#include "IParticleSystemSceneNode.h"
#include "irrArray.h"
#include "irrList.h"
#include "SMeshBuffer.h"
namespace irr
{
namespace scene
{
//! A particle system scene node.
/** A scene node controlling a particle system. The behavior of the particles
can be controlled by setting the right particle emitters and affectors.
*/
class CParticleSystemSceneNode2 : public ISceneNode
{
public:
        //! constructor
        CParticleSystemSceneNode2(ISceneNode* parent, ISceneManager* mgr, s32 id, ClientEnvironment *env, v3f rpos);
        //! destructor
        virtual ~CParticleSystemSceneNode2();
        //! Gets the particle emitter, which creates the particles.
        virtual IParticleEmitter* getEmitter() ;
        //! Sets the particle emitter, which creates the particles.
        virtual void setEmitter(IParticleEmitter* emitter) ;
        //! Adds new particle affector to the particle system.
        virtual void addAffector(IParticleAffector* affector) ;
        //! Get a list of all particle affectors.
        virtual const core::list<IParticleAffector*>& getAffectors() const ;
        //! Removes all particle affectors in the particle system.
        virtual void removeAllAffectors() ;
        //! Returns the material based on the zero based index i.
        virtual video::SMaterial& getMaterial(u32 i) ;
        //! Returns amount of materials used by this scene node.
        virtual u32 getMaterialCount() const ;
        //! pre render event
        virtual void OnRegisterSceneNode() ;
        //! render
        virtual void render() ;
        //! returns the axis aligned bounding box of this node
        virtual const core::aabbox3d<f32>& getBoundingBox() const ;

//        //! Creates a particle emitter for an animated mesh scene node
//        virtual IParticleAnimatedMeshSceneNodeEmitter* createAnimatedMeshSceneNodeEmitter(
//                        scene::IAnimatedMeshSceneNode* node, bool useNormalDirection = true,
//                        const core::vector3df& direction = core::vector3df(0.0f,0.03f,0.0f),
//                        f32 normalDirectionModifier = 100.0f, s32 mbNumber = -1,
//                        bool everyMeshVertex = false, u32 minParticlesPerSecond = 5,
//                        u32 maxParticlesPerSecond = 10,
//                        const video::SColor& minStartColor = video::SColor(255,0,0,0),
//                        const video::SColor& maxStartColor = video::SColor(255,255,255,255),
//                        u32 lifeTimeMin = 2000, u32 lifeTimeMax = 4000,
//                        s32 maxAngleDegrees = 0,
//                        const core::dimension2df& minStartSize = core::dimension2df(5.0f,5.0f),
//                        const core::dimension2df& maxStartSize = core::dimension2df(5.0f,5.0f) ) ;
//        //! Creates a box particle emitter.
//        virtual IParticleBoxEmitter* createBoxEmitter(
//                        const core::aabbox3df& box = core::aabbox3d<f32>(-10,0,-10,5,30,10),
//                        const core::vector3df& direction = core::vector3df(0.0f,0.03f,0.0f),
//                        u32 minParticlesPerSecond = 5,
//                        u32 maxParticlesPerSecond = 10,
//                        const video::SColor& minStartColor = video::SColor(255,0,0,0),
//                        const video::SColor& maxStartColor = video::SColor(255,255,255,255),
//                        u32 lifeTimeMin=2000, u32 lifeTimeMax=4000,
//                        s32 maxAngleDegrees=0,
//                        const core::dimension2df& minStartSize = core::dimension2df(5.0f,5.0f),
//                        const core::dimension2df& maxStartSize = core::dimension2df(5.0f,5.0f) ) ;
//        //! Creates a particle emitter for emitting from a cylinder
//        virtual IParticleCylinderEmitter* createCylinderEmitter(
//                        const core::vector3df& center, f32 radius,
//                        const core::vector3df& normal, f32 length,
//                        bool outlineOnly = false, const core::vector3df& direction = core::vector3df(0.0f,0.5f,0.0f),
//                        u32 minParticlesPerSecond = 5, u32 maxParticlesPersSecond = 10,
//                        const video::SColor& minStartColor = video::SColor(255,0,0,0),
//                        const video::SColor& maxStartColor = video::SColor(255,255,255,255),
//                        u32 lifeTimeMin = 2000, u32 lifeTimeMax = 4000,
//                        s32 maxAngleDegrees = 0,
//                        const core::dimension2df& minStartSize = core::dimension2df(5.0f,5.0f),
//                        const core::dimension2df& maxStartSize = core::dimension2df(5.0f,5.0f) ) ;
//        //! Creates a mesh particle emitter.
//        virtual IParticleMeshEmitter* createMeshEmitter(
//                        scene::IMesh* mesh, bool useNormalDirection = true,
//                        const core::vector3df& direction = core::vector3df(0.0f,0.03f,0.0f),
//                        f32 normalDirectionModifier = 100.0f, s32 mbNumber = -1,
//                        bool everyMeshVertex = false,
//                        u32 minParticlesPerSecond = 5,
//                        u32 maxParticlesPerSecond = 10,
//                        const video::SColor& minStartColor = video::SColor(255,0,0,0),
//                        const video::SColor& maxStartColor = video::SColor(255,255,255,255),
//                        u32 lifeTimeMin = 2000, u32 lifeTimeMax = 4000,
//                        s32 maxAngleDegrees = 0,
//                        const core::dimension2df& minStartSize = core::dimension2df(5.0f,5.0f),
//                        const core::dimension2df& maxStartSize = core::dimension2df(5.0f,5.0f) ) ;
//        //! Creates a point particle emitter.
//        virtual IParticlePointEmitter* createPointEmitter(
//                        const core::vector3df& direction = core::vector3df(0.0f,0.03f,0.0f),
//                        u32 minParticlesPerSecond = 5,
//                        u32 maxParticlesPerSecond = 10,
//                        const video::SColor& minStartColor = video::SColor(255,0,0,0),
//                        const video::SColor& maxStartColor = video::SColor(255,255,255,255),
//                        u32 lifeTimeMin=2000, u32 lifeTimeMax=4000,
//                        s32 maxAngleDegrees=0,
//                        const core::dimension2df& minStartSize = core::dimension2df(5.0f,5.0f),
//                        const core::dimension2df& maxStartSize = core::dimension2df(5.0f,5.0f) ) ;
//        //! Creates a ring particle emitter.
//        virtual IParticleRingEmitter* createRingEmitter(
//                        const core::vector3df& center, f32 radius, f32 ringThickness,
//                        const core::vector3df& direction = core::vector3df(0.0f,0.03f,0.0f),
//                        u32 minParticlesPerSecond = 5,
//                        u32 maxParticlesPerSecond = 10,
//                        const video::SColor& minStartColor = video::SColor(255,0,0,0),
//                        const video::SColor& maxStartColor = video::SColor(255,255,255,255),
//                        u32 lifeTimeMin=2000, u32 lifeTimeMax=4000,
//                        s32 maxAngleDegrees=0,
//                        const core::dimension2df& minStartSize = core::dimension2df(5.0f,5.0f),
//                        const core::dimension2df& maxStartSize = core::dimension2df(5.0f,5.0f) ) ;
//        //! Creates a sphere particle emitter.
//        virtual IParticleSphereEmitter* createSphereEmitter(
//                        const core::vector3df& center, f32 radius,
//                        const core::vector3df& direction = core::vector3df(0.0f,0.03f,0.0f),
//                        u32 minParticlesPerSecond = 5,
//                        u32 maxParticlesPerSecond = 10,
//                        const video::SColor& minStartColor = video::SColor(255,0,0,0),
//                        const video::SColor& maxStartColor = video::SColor(255,255,255,255),
//                        u32 lifeTimeMin=2000, u32 lifeTimeMax=4000,
//                        s32 maxAngleDegrees=0,
//                        const core::dimension2df& minStartSize = core::dimension2df(5.0f,5.0f),
//                        const core::dimension2df& maxStartSize = core::dimension2df(5.0f,5.0f) ) ;
//        //! Creates a point attraction affector. This affector modifies the positions of the
//        //! particles and attracts them to a specified point at a specified speed per second.
//        virtual IParticleAttractionAffector* createAttractionAffector(
//                        const core::vector3df& point, f32 speed = 1.0f, bool attract = true,
//                        bool affectX = true, bool affectY = true, bool affectZ = true) ;
//        //! Creates a scale particle affector.
//        virtual IParticleAffector* createScaleParticleAffector(
//                        const core::dimension2df& scaleTo = core::dimension2df(1.0f, 1.0f)) ;
//        //! Creates a fade out particle affector.
//        virtual IParticleFadeOutAffector* createFadeOutParticleAffector(
//                        const video::SColor& targetColor = video::SColor(0,0,0,0),
//                        u32 timeNeededToFadeOut = 1000) ;
//        //! Creates a gravity affector.
//        virtual IParticleGravityAffector* createGravityAffector(
//                        const core::vector3df& gravity = core::vector3df(0.0f,-0.03f,0.0f),
//                        u32 timeForceLost = 1000) ;
//        //! Creates a rotation affector. This affector rotates the particles
//        //! around a specified pivot point. The speed is in Degrees per second.
//        virtual IParticleRotationAffector* createRotationAffector(
//                        const core::vector3df& speed = core::vector3df(5.0f,5.0f,5.0f),
//                        const core::vector3df& pivotPoint = core::vector3df(0.0f,0.0f,0.0f) ) ;
        virtual void setPosition(const core::vector3df newpos);
        virtual void upd(v3s16 camera_offset);
        //! Sets the size of all particles.
        virtual void setParticleSize(
                        const core::dimension2d<f32> &size = core::dimension2d<f32>(5.0f, 5.0f)) ;
        //! Sets if the particles should be global. If they are, the particles are affected by
        //! the movement of the particle system scene node too, otherwise they completely
        //! ignore it. Default is true.
        virtual void setParticlesAreGlobal(bool global=true) ;
        //! Remove all currently visible particles
        virtual void clearParticles() ;
        //! Do manually update the particles.
        //! This should only be called when you want to render the node outside the scenegraph,
        //! as the node will care about this otherwise automatically.
        virtual void doParticleSystem(u32 time) ;
        //! Writes attributes of the scene node.
//        virtual void serializeAttributes(io::IAttributes* out, io::SAttributeReadWriteOptions* options=0) const ;
//        //! Reads attributes of the scene node.
//        virtual void deserializeAttributes(io::IAttributes* in, io::SAttributeReadWriteOptions* options=0) ;
        //! Returns type of the scene node
        virtual ESCENE_NODE_TYPE getType() const  { return ESNT_PARTICLE_SYSTEM; }
private:
        void reallocateBuffers();
        core::list<IParticleAffector*> AffectorList;
        IParticleEmitter* Emitter;
        core::array<SParticle> Particles;
        core::dimension2d<f32> ParticleSize;
        u32 LastEmitTime;
        core::matrix4 LastAbsoluteTransformation;
        SMeshBuffer* Buffer;

        enum E_PARTICLES_PRIMITIVE
        {
                EPP_POINT=0,
                EPP_BILLBOARD,
                EPP_POINTSPRITE
        };
        E_PARTICLES_PRIMITIVE ParticlePrimitive;

        bool ParticlesAreGlobal;
        ClientEnvironment *m_env;
        v3f rpos;
        v3s16 old_camera_offset;
        bool first_run;
};
} // end namespace scene
} // end namespace irr
#endif
