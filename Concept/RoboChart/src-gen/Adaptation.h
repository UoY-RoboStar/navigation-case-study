#ifndef ROBOCALC_CONTROLLERS_ADAPTATION_H_
#define ROBOCALC_CONTROLLERS_ADAPTATION_H_

#include "Turtlebot.h"
#include "RoboCalcAPI/Controller.h"
#include "DataTypes.h"

#include "Adaptation_Knowledge.h"
#include "Adaptation_Plan.h"
#include "Adaptation_Execute.h"
#include "Adaptation_Analyse.h"
#include "Adaptation_Legitimate.h"
#include "Adaptation_Monitor.h"

class Adaptation: public robocalc::Controller 
{
public:
	Adaptation(Turtlebot& _platform) : platform(&_platform){};
	Adaptation() : platform(nullptr){};
	
	~Adaptation() = default;
	
	void Execute()
	{
		adaptation_Knowledge.execute();
		adaptation_Plan.execute();
		adaptation_Execute.execute();
		adaptation_Analyse.execute();
		adaptation_Legitimate.execute();
		adaptation_Monitor.execute();
	}
	
	struct Channels
	{
		Adaptation& instance;
		Channels(Adaptation& _instance) : instance(_instance) {}
		
		EventBuffer* tryEmitLidarMasks_Plan(void* sender, std::tuple<std::vector<BoolLidarMask>> args)
		{
			if(instance.adaptation_Plan.canReceiveLidarMasks(args))
				instance.adaptation_Plan.lidarMasks_in.trigger(sender, args);
				
			return &instance.adaptation_Plan.lidarMasks_in;
		}
		
		EventBuffer* tryEmitGet_lidarMasks(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_lidarMasks_Legitimate(args))
				instance.adaptation_Knowledge.get_lidarMasks_Legitimate_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_lidarMasks_Legitimate_in;
		}
		
		EventBuffer* tryEmitPlannedLidarMask(void* sender, std::tuple<BoolLidarMask> args)
		{
			if(instance.adaptation_Analyse.canReceivePlannedLidarMask(args))
				instance.adaptation_Analyse.plannedLidarMask_in.trigger(sender, args);
				
			return &instance.adaptation_Analyse.plannedLidarMask_in;
		}
		
		EventBuffer* tryEmitGet_laserScans(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_laserScans_Legitimate(args))
				instance.adaptation_Knowledge.get_laserScans_Legitimate_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_laserScans_Legitimate_in;
		}
		
		EventBuffer* tryEmitGet_laserScans(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_laserScans(args))
				instance.adaptation_Knowledge.get_laserScans_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_laserScans_in;
		}
		
		EventBuffer* tryEmitSet_lidarMasks(void* sender, std::tuple<std::vector<BoolLidarMask>> args)
		{
			if(instance.adaptation_Knowledge.canReceiveSet_lidarMasks(args))
				instance.adaptation_Knowledge.set_lidarMasks_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.set_lidarMasks_in;
		}
		
		EventBuffer* tryEmitPlanAccepted(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Execute.canReceiveExecutePlan(args))
				instance.adaptation_Execute.executePlan_in.trigger(sender, args);
				
			return &instance.adaptation_Execute.executePlan_in;
		}
		
		EventBuffer* tryEmitPlannedLidarMask_Legitimate(void* sender, std::tuple<BoolLidarMask> args)
		{
			if(instance.adaptation_Legitimate.canReceivePlannedLidarMask(args))
				instance.adaptation_Legitimate.plannedLidarMask_in.trigger(sender, args);
				
			return &instance.adaptation_Legitimate.plannedLidarMask_in;
		}
		
		EventBuffer* tryEmitPlanRejected(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Plan.canReceivePlanRejected(args))
				instance.adaptation_Plan.planRejected_in.trigger(sender, args);
				
			return &instance.adaptation_Plan.planRejected_in;
		}
		
		EventBuffer* tryEmitSet_laserScans(void* sender, std::tuple<std::vector<LaserScan>> args)
		{
			if(instance.adaptation_Knowledge.canReceiveSet_laserScans(args))
				instance.adaptation_Knowledge.set_laserScans_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.set_laserScans_in;
		}
		
		EventBuffer* tryEmitLaserScans_Legitimate(void* sender, std::tuple<std::vector<LaserScan>> args)
		{
			if(instance.adaptation_Legitimate.canReceiveLaserScans(args))
				instance.adaptation_Legitimate.laserScans_in.trigger(sender, args);
				
			return &instance.adaptation_Legitimate.laserScans_in;
		}
		
		EventBuffer* tryEmitLaserScans_Analyse(void* sender, std::tuple<std::vector<LaserScan>> args)
		{
			if(instance.adaptation_Analyse.canReceiveLaserScans(args))
				instance.adaptation_Analyse.laserScans_in.trigger(sender, args);
				
			return &instance.adaptation_Analyse.laserScans_in;
		}
		
		EventBuffer* tryEmitGet_laserScans(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_laserScans_Analyse(args))
				instance.adaptation_Knowledge.get_laserScans_Analyse_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_laserScans_Analyse_in;
		}
		
		EventBuffer* tryEmitGet_lidarMasks(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_lidarMasks_Plan(args))
				instance.adaptation_Knowledge.get_lidarMasks_Plan_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_lidarMasks_Plan_in;
		}
		
		EventBuffer* tryEmitLidarMasks(void* sender, std::tuple<std::vector<BoolLidarMask>> args)
		{
			if(instance.adaptation_Analyse.canReceiveLidarMasks(args))
				instance.adaptation_Analyse.lidarMasks_in.trigger(sender, args);
				
			return &instance.adaptation_Analyse.lidarMasks_in;
		}
		
		EventBuffer* tryEmitSet_plannedLidarMask(void* sender, std::tuple<BoolLidarMask> args)
		{
			if(instance.adaptation_Knowledge.canReceiveSet_plannedLidarMask(args))
				instance.adaptation_Knowledge.set_plannedLidarMask_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.set_plannedLidarMask_in;
		}
		
		EventBuffer* tryEmitProcessedData(void* sender, std::tuple<LaserScan> args)
		{
			if(instance.adaptation_Analyse.canReceiveProcessedData(args))
				instance.adaptation_Analyse.processedData_in.trigger(sender, args);
				
			return &instance.adaptation_Analyse.processedData_in;
		}
		
		EventBuffer* tryEmitDirections(void* sender, std::tuple<SpinConfig> args)
		{
			if(instance.adaptation_Plan.canReceiveDirections(args))
				instance.adaptation_Plan.directions_in.trigger(sender, args);
				
			return &instance.adaptation_Plan.directions_in;
		}
		
		EventBuffer* tryEmitGet_plannedLidarMask(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_plannedLidarMask_Legitimate(args))
				instance.adaptation_Knowledge.get_plannedLidarMask_Legitimate_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_plannedLidarMask_Legitimate_in;
		}
		
		EventBuffer* tryEmitSpinConfig(void* sender, std::tuple<SpinConfig> args)
		{
			spinConfig_in.trigger(sender, args);
			return &spinConfig_in;
		}
		
		EventBuffer* tryEmitLidarMasks_Legitimate(void* sender, std::tuple<std::vector<BoolLidarMask>> args)
		{
			if(instance.adaptation_Legitimate.canReceiveLidarMasks(args))
				instance.adaptation_Legitimate.lidarMasks_in.trigger(sender, args);
				
			return &instance.adaptation_Legitimate.lidarMasks_in;
		}
		
		EventBuffer* tryEmitGet_plannedLidarMask(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_plannedLidarMask(args))
				instance.adaptation_Knowledge.get_plannedLidarMask_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_plannedLidarMask_in;
		}
		
		EventBuffer* tryEmitGet_laserScans(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_laserScans_Plan(args))
				instance.adaptation_Knowledge.get_laserScans_Plan_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_laserScans_Plan_in;
		}
		
		EventBuffer* tryEmitPlanningCompleted(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Legitimate.canReceiveVerifyPlan(args))
				instance.adaptation_Legitimate.verifyPlan_in.trigger(sender, args);
				
			return &instance.adaptation_Legitimate.verifyPlan_in;
		}
		
		EventBuffer* tryEmitLaserScans_Plan(void* sender, std::tuple<std::vector<LaserScan>> args)
		{
			if(instance.adaptation_Plan.canReceiveLaserScans(args))
				instance.adaptation_Plan.laserScans_in.trigger(sender, args);
				
			return &instance.adaptation_Plan.laserScans_in;
		}
		
		EventBuffer* tryEmitGet_directions(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_directions_Legitimate(args))
				instance.adaptation_Knowledge.get_directions_Legitimate_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_directions_Legitimate_in;
		}
		
		EventBuffer* tryEmitLaserScans(void* sender, std::tuple<std::vector<LaserScan>> args)
		{
			if(instance.adaptation_Monitor.canReceiveLaserScans(args))
				instance.adaptation_Monitor.laserScans_in.trigger(sender, args);
				
			return &instance.adaptation_Monitor.laserScans_in;
		}
		
		EventBuffer* tryEmitAdaptationCompleted(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Analyse.canReceiveAdaptationCompleted(args))
				instance.adaptation_Analyse.adaptationCompleted_in.trigger(sender, args);
				
			return &instance.adaptation_Analyse.adaptationCompleted_in;
		}
		
		EventBuffer* tryEmitSet_directions(void* sender, std::tuple<SpinConfig> args)
		{
			if(instance.adaptation_Knowledge.canReceiveSet_directions(args))
				instance.adaptation_Knowledge.set_directions_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.set_directions_in;
		}
		
		EventBuffer* tryEmitGet_plannedLidarMask(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_plannedLidarMask_Plan(args))
				instance.adaptation_Knowledge.get_plannedLidarMask_Plan_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_plannedLidarMask_Plan_in;
		}
		
		EventBuffer* tryEmitLaserScan(void* sender, std::tuple<LaserScan> args)
		{
			if(instance.adaptation_Monitor.canReceiveLaserScan(args))
				instance.adaptation_Monitor.laserScan_in.trigger(sender, args);
				
			return &instance.adaptation_Monitor.laserScan_in;
		}
		
		EventBuffer* tryEmitDirections_Legitimate(void* sender, std::tuple<SpinConfig> args)
		{
			if(instance.adaptation_Legitimate.canReceiveDirections(args))
				instance.adaptation_Legitimate.directions_in.trigger(sender, args);
				
			return &instance.adaptation_Legitimate.directions_in;
		}
		
		EventBuffer* tryEmitAnomalyFound(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Plan.canReceiveRequestPlan(args))
				instance.adaptation_Plan.requestPlan_in.trigger(sender, args);
				
			return &instance.adaptation_Plan.requestPlan_in;
		}
		
		EventBuffer* tryEmitGet_lidarMasks(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_lidarMasks(args))
				instance.adaptation_Knowledge.get_lidarMasks_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_lidarMasks_in;
		}
		
		EventBuffer* tryEmitGet_directions(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_directions(args))
				instance.adaptation_Knowledge.get_directions_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_directions_in;
		}
		
		EventBuffer* tryEmitPlannedLidarMask_Plan(void* sender, std::tuple<BoolLidarMask> args)
		{
			if(instance.adaptation_Plan.canReceivePlannedLidarMask(args))
				instance.adaptation_Plan.plannedLidarMask_in.trigger(sender, args);
				
			return &instance.adaptation_Plan.plannedLidarMask_in;
		}
		
		EventBuffer* tryEmitDirections_Execute(void* sender, std::tuple<SpinConfig> args)
		{
			if(instance.adaptation_Execute.canReceiveDirections(args))
				instance.adaptation_Execute.directions_in.trigger(sender, args);
				
			return &instance.adaptation_Execute.directions_in;
		}
		
		EventBuffer* tryEmitGet_directions(void* sender, std::tuple<> args)
		{
			if(instance.adaptation_Knowledge.canReceiveGet_directions_Execute(args))
				instance.adaptation_Knowledge.get_directions_Execute_in.trigger(sender, args);
				
			return &instance.adaptation_Knowledge.get_directions_Execute_in;
		}
		
		struct SpinConfig_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<SpinConfig> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<SpinConfig> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} spinConfig_in;
	};
	
	Channels channels{*this};
	
	Turtlebot* platform;
	Adaptation_Knowledge_StateMachine<Adaptation> adaptation_Knowledge{*platform, *this, &adaptation_Knowledge};
	Adaptation_Plan_StateMachine<Adaptation> adaptation_Plan{*platform, *this, &adaptation_Plan};
	Adaptation_Execute_StateMachine<Adaptation> adaptation_Execute{*platform, *this, &adaptation_Execute};
	Adaptation_Analyse_StateMachine<Adaptation> adaptation_Analyse{*platform, *this, &adaptation_Analyse};
	Adaptation_Legitimate_StateMachine<Adaptation> adaptation_Legitimate{*platform, *this, &adaptation_Legitimate};
	Adaptation_Monitor_StateMachine<Adaptation> adaptation_Monitor{*platform, *this, &adaptation_Monitor};
};

#endif
