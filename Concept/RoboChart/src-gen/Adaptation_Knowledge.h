#ifndef ROBOCALC_STATEMACHINES_ADAPTATION_KNOWLEDGE_H_
#define ROBOCALC_STATEMACHINES_ADAPTATION_KNOWLEDGE_H_

#include "RoboCalcAPI/StateMachine.h"

#ifndef ROBOCALC_THREAD_SAFE
#define THREAD_SAFE_ONLY(x)
#else
#include <mutex>
#define THREAD_SAFE_ONLY(x) x
#endif

#include "Turtlebot.h"
#include "RoboCalcAPI/Timer.h"
#include "Functions.h"
#include "DataTypes.h"
#include <assert.h>
#include <set>

using namespace robocalc;
using namespace robocalc::functions;

template<typename ControllerType>
class Adaptation_Knowledge_StateMachine : public StateMachine<Turtlebot, ControllerType, Adaptation_Knowledge_StateMachine<ControllerType>>
{
	public:
		ControllerType& controller;
	public:
		class Knowledge_State_t : public robocalc::State<ControllerType>
		{
			public:
				explicit Knowledge_State_t(ControllerType& _controller, Adaptation_Knowledge_StateMachine<ControllerType>* topLevel) 
					: robocalc::State<ControllerType>(_controller)
				{
				}
				
				Knowledge_State_t() = delete;
			
				void enter() override
				{
				}
				
				void exit() override
				{
				}
				
				void during() override
				{
				}
		};
		
		Knowledge_State_t knowledge_State;
		
		enum PossibleStates
		{
			s_Knowledge,
			j_i0
		} currentState = j_i0; 

	public:
		Adaptation_Knowledge_StateMachine(Turtlebot& _platform, ControllerType& _controller, Adaptation_Knowledge_StateMachine<ControllerType>* topLevel = nullptr) 
			: StateMachine<Turtlebot, ControllerType, Adaptation_Knowledge_StateMachine<ControllerType>>(_platform, _controller, topLevel), controller{_controller}
			,
			knowledge_State{_controller, this}
			{};
		Adaptation_Knowledge_StateMachine() = delete;
		~Adaptation_Knowledge_StateMachine() = default;
	
	
		bool canReceiveGet_laserScans(std::tuple<> args)
		{
			if(blockedByGet_laserScans != nullptr)
				if(blockedByGet_laserScans->getSender() == this)
					return false;
												
			blockedByGet_laserScans = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_laserScans = nullptr;
															
		inline void tryEmitGet_laserScans(std::tuple<> args)
		{
			blockedByGet_laserScans = controller.channels.tryEmitGet_laserScans(this, args);
		}
		
		struct Get_laserScans_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_laserScans_in;
		
		bool canReceiveSet_laserScans(std::tuple<std::vector<LaserScan>> args)
		{
			if(blockedBySet_laserScans != nullptr)
				if(blockedBySet_laserScans->getSender() == this)
					return false;
												
			blockedBySet_laserScans = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedBySet_laserScans = nullptr;
															
		inline void tryEmitSet_laserScans(std::tuple<std::vector<LaserScan>> args)
		{
			blockedBySet_laserScans = controller.channels.tryEmitSet_laserScans(this, args);
		}
		
		struct Set_laserScans_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<LaserScan>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<LaserScan>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} set_laserScans_in;
		
		bool canReceiveGet_laserScans_ext(std::tuple<> args)
		{
			if(blockedByGet_laserScans_ext != nullptr)
				if(blockedByGet_laserScans_ext->getSender() == this)
					return false;
												
			blockedByGet_laserScans_ext = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_laserScans_ext = nullptr;
															
		inline void tryEmitGet_laserScans_ext(std::tuple<> args)
		{
			blockedByGet_laserScans_ext = controller.channels.tryEmitGet_laserScans_ext(this, args);
		}
		
		struct Get_laserScans_ext_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_laserScans_ext_in;
		
		bool canReceiveGet_laserScans_Analyse(std::tuple<> args)
		{
			if(blockedByGet_laserScans_Analyse != nullptr)
				if(blockedByGet_laserScans_Analyse->getSender() == this)
					return false;
												
			blockedByGet_laserScans_Analyse = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_laserScans_Analyse = nullptr;
															
		inline void tryEmitGet_laserScans_Analyse(std::tuple<> args)
		{
			blockedByGet_laserScans_Analyse = controller.channels.tryEmitGet_laserScans_Analyse(this, args);
		}
		
		struct Get_laserScans_Analyse_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_laserScans_Analyse_in;
		
		bool canReceiveGet_laserScans_Plan(std::tuple<> args)
		{
			if(blockedByGet_laserScans_Plan != nullptr)
				if(blockedByGet_laserScans_Plan->getSender() == this)
					return false;
												
			blockedByGet_laserScans_Plan = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_laserScans_Plan = nullptr;
															
		inline void tryEmitGet_laserScans_Plan(std::tuple<> args)
		{
			blockedByGet_laserScans_Plan = controller.channels.tryEmitGet_laserScans_Plan(this, args);
		}
		
		struct Get_laserScans_Plan_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_laserScans_Plan_in;
		
		bool canReceiveGet_laserScans_Legitimate(std::tuple<> args)
		{
			if(blockedByGet_laserScans_Legitimate != nullptr)
				if(blockedByGet_laserScans_Legitimate->getSender() == this)
					return false;
												
			blockedByGet_laserScans_Legitimate = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_laserScans_Legitimate = nullptr;
															
		inline void tryEmitGet_laserScans_Legitimate(std::tuple<> args)
		{
			blockedByGet_laserScans_Legitimate = controller.channels.tryEmitGet_laserScans_Legitimate(this, args);
		}
		
		struct Get_laserScans_Legitimate_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_laserScans_Legitimate_in;
		
		bool canReceiveGet_lidarMasks(std::tuple<> args)
		{
			if(blockedByGet_lidarMasks != nullptr)
				if(blockedByGet_lidarMasks->getSender() == this)
					return false;
												
			blockedByGet_lidarMasks = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_lidarMasks = nullptr;
															
		inline void tryEmitGet_lidarMasks(std::tuple<> args)
		{
			blockedByGet_lidarMasks = controller.channels.tryEmitGet_lidarMasks(this, args);
		}
		
		struct Get_lidarMasks_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_lidarMasks_in;
		
		bool canReceiveSet_lidarMasks(std::tuple<std::vector<BoolLidarMask>> args)
		{
			if(blockedBySet_lidarMasks != nullptr)
				if(blockedBySet_lidarMasks->getSender() == this)
					return false;
												
			blockedBySet_lidarMasks = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedBySet_lidarMasks = nullptr;
															
		inline void tryEmitSet_lidarMasks(std::tuple<std::vector<BoolLidarMask>> args)
		{
			blockedBySet_lidarMasks = controller.channels.tryEmitSet_lidarMasks(this, args);
		}
		
		struct Set_lidarMasks_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<BoolLidarMask>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<BoolLidarMask>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} set_lidarMasks_in;
		
		bool canReceiveGet_lidarMasks_ext(std::tuple<> args)
		{
			if(blockedByGet_lidarMasks_ext != nullptr)
				if(blockedByGet_lidarMasks_ext->getSender() == this)
					return false;
												
			blockedByGet_lidarMasks_ext = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_lidarMasks_ext = nullptr;
															
		inline void tryEmitGet_lidarMasks_ext(std::tuple<> args)
		{
			blockedByGet_lidarMasks_ext = controller.channels.tryEmitGet_lidarMasks_ext(this, args);
		}
		
		struct Get_lidarMasks_ext_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_lidarMasks_ext_in;
		
		bool canReceiveGet_plannedLidarMask(std::tuple<> args)
		{
			if(blockedByGet_plannedLidarMask != nullptr)
				if(blockedByGet_plannedLidarMask->getSender() == this)
					return false;
												
			blockedByGet_plannedLidarMask = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_plannedLidarMask = nullptr;
															
		inline void tryEmitGet_plannedLidarMask(std::tuple<> args)
		{
			blockedByGet_plannedLidarMask = controller.channels.tryEmitGet_plannedLidarMask(this, args);
		}
		
		struct Get_plannedLidarMask_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_plannedLidarMask_in;
		
		bool canReceiveSet_plannedLidarMask(std::tuple<BoolLidarMask> args)
		{
			if(blockedBySet_plannedLidarMask != nullptr)
				if(blockedBySet_plannedLidarMask->getSender() == this)
					return false;
												
			blockedBySet_plannedLidarMask = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedBySet_plannedLidarMask = nullptr;
															
		inline void tryEmitSet_plannedLidarMask(std::tuple<BoolLidarMask> args)
		{
			blockedBySet_plannedLidarMask = controller.channels.tryEmitSet_plannedLidarMask(this, args);
		}
		
		struct Set_plannedLidarMask_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<BoolLidarMask> _args;
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
			
			void trigger(void* sender, std::tuple<BoolLidarMask> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} set_plannedLidarMask_in;
		
		bool canReceiveGet_plannedLidarMask_ext(std::tuple<> args)
		{
			if(blockedByGet_plannedLidarMask_ext != nullptr)
				if(blockedByGet_plannedLidarMask_ext->getSender() == this)
					return false;
												
			blockedByGet_plannedLidarMask_ext = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_plannedLidarMask_ext = nullptr;
															
		inline void tryEmitGet_plannedLidarMask_ext(std::tuple<> args)
		{
			blockedByGet_plannedLidarMask_ext = controller.channels.tryEmitGet_plannedLidarMask_ext(this, args);
		}
		
		struct Get_plannedLidarMask_ext_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_plannedLidarMask_ext_in;
		
		bool canReceiveGet_lidarMasks_Plan(std::tuple<> args)
		{
			if(blockedByGet_lidarMasks_Plan != nullptr)
				if(blockedByGet_lidarMasks_Plan->getSender() == this)
					return false;
												
			blockedByGet_lidarMasks_Plan = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_lidarMasks_Plan = nullptr;
															
		inline void tryEmitGet_lidarMasks_Plan(std::tuple<> args)
		{
			blockedByGet_lidarMasks_Plan = controller.channels.tryEmitGet_lidarMasks_Plan(this, args);
		}
		
		struct Get_lidarMasks_Plan_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_lidarMasks_Plan_in;
		
		bool canReceiveGet_plannedLidarMask_Plan(std::tuple<> args)
		{
			if(blockedByGet_plannedLidarMask_Plan != nullptr)
				if(blockedByGet_plannedLidarMask_Plan->getSender() == this)
					return false;
												
			blockedByGet_plannedLidarMask_Plan = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_plannedLidarMask_Plan = nullptr;
															
		inline void tryEmitGet_plannedLidarMask_Plan(std::tuple<> args)
		{
			blockedByGet_plannedLidarMask_Plan = controller.channels.tryEmitGet_plannedLidarMask_Plan(this, args);
		}
		
		struct Get_plannedLidarMask_Plan_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_plannedLidarMask_Plan_in;
		
		bool canReceiveGet_lidarMasks_Legitimate(std::tuple<> args)
		{
			if(blockedByGet_lidarMasks_Legitimate != nullptr)
				if(blockedByGet_lidarMasks_Legitimate->getSender() == this)
					return false;
												
			blockedByGet_lidarMasks_Legitimate = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_lidarMasks_Legitimate = nullptr;
															
		inline void tryEmitGet_lidarMasks_Legitimate(std::tuple<> args)
		{
			blockedByGet_lidarMasks_Legitimate = controller.channels.tryEmitGet_lidarMasks_Legitimate(this, args);
		}
		
		struct Get_lidarMasks_Legitimate_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_lidarMasks_Legitimate_in;
		
		bool canReceiveGet_plannedLidarMask_Legitimate(std::tuple<> args)
		{
			if(blockedByGet_plannedLidarMask_Legitimate != nullptr)
				if(blockedByGet_plannedLidarMask_Legitimate->getSender() == this)
					return false;
												
			blockedByGet_plannedLidarMask_Legitimate = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_plannedLidarMask_Legitimate = nullptr;
															
		inline void tryEmitGet_plannedLidarMask_Legitimate(std::tuple<> args)
		{
			blockedByGet_plannedLidarMask_Legitimate = controller.channels.tryEmitGet_plannedLidarMask_Legitimate(this, args);
		}
		
		struct Get_plannedLidarMask_Legitimate_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_plannedLidarMask_Legitimate_in;
		
		bool canReceiveGet_directions(std::tuple<> args)
		{
			if(blockedByGet_directions != nullptr)
				if(blockedByGet_directions->getSender() == this)
					return false;
												
			blockedByGet_directions = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_directions = nullptr;
															
		inline void tryEmitGet_directions(std::tuple<> args)
		{
			blockedByGet_directions = controller.channels.tryEmitGet_directions(this, args);
		}
		
		struct Get_directions_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_directions_in;
		
		bool canReceiveSet_directions(std::tuple<SpinConfig> args)
		{
			if(blockedBySet_directions != nullptr)
				if(blockedBySet_directions->getSender() == this)
					return false;
												
			blockedBySet_directions = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedBySet_directions = nullptr;
															
		inline void tryEmitSet_directions(std::tuple<SpinConfig> args)
		{
			blockedBySet_directions = controller.channels.tryEmitSet_directions(this, args);
		}
		
		struct Set_directions_t : public EventBuffer
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
		} set_directions_in;
		
		bool canReceiveGet_directions_ext(std::tuple<> args)
		{
			if(blockedByGet_directions_ext != nullptr)
				if(blockedByGet_directions_ext->getSender() == this)
					return false;
												
			blockedByGet_directions_ext = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_directions_ext = nullptr;
															
		inline void tryEmitGet_directions_ext(std::tuple<> args)
		{
			blockedByGet_directions_ext = controller.channels.tryEmitGet_directions_ext(this, args);
		}
		
		struct Get_directions_ext_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_directions_ext_in;
		
		bool canReceiveGet_directions_Legitimate(std::tuple<> args)
		{
			if(blockedByGet_directions_Legitimate != nullptr)
				if(blockedByGet_directions_Legitimate->getSender() == this)
					return false;
												
			blockedByGet_directions_Legitimate = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_directions_Legitimate = nullptr;
															
		inline void tryEmitGet_directions_Legitimate(std::tuple<> args)
		{
			blockedByGet_directions_Legitimate = controller.channels.tryEmitGet_directions_Legitimate(this, args);
		}
		
		struct Get_directions_Legitimate_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_directions_Legitimate_in;
		
		bool canReceiveGet_directions_Execute(std::tuple<> args)
		{
			if(blockedByGet_directions_Execute != nullptr)
				if(blockedByGet_directions_Execute->getSender() == this)
					return false;
												
			blockedByGet_directions_Execute = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					return true;
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByGet_directions_Execute = nullptr;
															
		inline void tryEmitGet_directions_Execute(std::tuple<> args)
		{
			blockedByGet_directions_Execute = controller.channels.tryEmitGet_directions_Execute(this, args);
		}
		
		struct Get_directions_Execute_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
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
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} get_directions_Execute_in;
		
		bool canReceiveLaserScans(std::tuple<std::vector<LaserScan>> args)
		{
			if(blockedByLaserScans != nullptr)
				if(blockedByLaserScans->getSender() == this)
					return false;
												
			blockedByLaserScans = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByLaserScans = nullptr;
															
		inline void tryEmitLaserScans(std::tuple<std::vector<LaserScan>> args)
		{
			blockedByLaserScans = controller.channels.tryEmitLaserScans(this, args);
		}
		
		struct LaserScans_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<LaserScan>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<LaserScan>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} laserScans_in;
		
		bool canReceiveLaserScans_ext(std::tuple<std::vector<LaserScan>> args)
		{
			if(blockedByLaserScans_ext != nullptr)
				if(blockedByLaserScans_ext->getSender() == this)
					return false;
												
			blockedByLaserScans_ext = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByLaserScans_ext = nullptr;
															
		inline void tryEmitLaserScans_ext(std::tuple<std::vector<LaserScan>> args)
		{
			blockedByLaserScans_ext = controller.channels.tryEmitLaserScans_ext(this, args);
		}
		
		struct LaserScans_ext_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<LaserScan>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<LaserScan>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} laserScans_ext_in;
		
		bool canReceiveLaserScans_Analyse(std::tuple<std::vector<LaserScan>> args)
		{
			if(blockedByLaserScans_Analyse != nullptr)
				if(blockedByLaserScans_Analyse->getSender() == this)
					return false;
												
			blockedByLaserScans_Analyse = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByLaserScans_Analyse = nullptr;
															
		inline void tryEmitLaserScans_Analyse(std::tuple<std::vector<LaserScan>> args)
		{
			blockedByLaserScans_Analyse = controller.channels.tryEmitLaserScans_Analyse(this, args);
		}
		
		struct LaserScans_Analyse_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<LaserScan>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<LaserScan>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} laserScans_Analyse_in;
		
		bool canReceiveLaserScans_Plan(std::tuple<std::vector<LaserScan>> args)
		{
			if(blockedByLaserScans_Plan != nullptr)
				if(blockedByLaserScans_Plan->getSender() == this)
					return false;
												
			blockedByLaserScans_Plan = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByLaserScans_Plan = nullptr;
															
		inline void tryEmitLaserScans_Plan(std::tuple<std::vector<LaserScan>> args)
		{
			blockedByLaserScans_Plan = controller.channels.tryEmitLaserScans_Plan(this, args);
		}
		
		struct LaserScans_Plan_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<LaserScan>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<LaserScan>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} laserScans_Plan_in;
		
		bool canReceiveLaserScans_Legitimate(std::tuple<std::vector<LaserScan>> args)
		{
			if(blockedByLaserScans_Legitimate != nullptr)
				if(blockedByLaserScans_Legitimate->getSender() == this)
					return false;
												
			blockedByLaserScans_Legitimate = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByLaserScans_Legitimate = nullptr;
															
		inline void tryEmitLaserScans_Legitimate(std::tuple<std::vector<LaserScan>> args)
		{
			blockedByLaserScans_Legitimate = controller.channels.tryEmitLaserScans_Legitimate(this, args);
		}
		
		struct LaserScans_Legitimate_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<LaserScan>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<LaserScan>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} laserScans_Legitimate_in;
		
		bool canReceiveLidarMasks(std::tuple<std::vector<BoolLidarMask>> args)
		{
			if(blockedByLidarMasks != nullptr)
				if(blockedByLidarMasks->getSender() == this)
					return false;
												
			blockedByLidarMasks = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByLidarMasks = nullptr;
															
		inline void tryEmitLidarMasks(std::tuple<std::vector<BoolLidarMask>> args)
		{
			blockedByLidarMasks = controller.channels.tryEmitLidarMasks(this, args);
		}
		
		struct LidarMasks_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<BoolLidarMask>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<BoolLidarMask>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} lidarMasks_in;
		
		bool canReceivePlannedLidarMask(std::tuple<BoolLidarMask> args)
		{
			if(blockedByPlannedLidarMask != nullptr)
				if(blockedByPlannedLidarMask->getSender() == this)
					return false;
												
			blockedByPlannedLidarMask = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByPlannedLidarMask = nullptr;
															
		inline void tryEmitPlannedLidarMask(std::tuple<BoolLidarMask> args)
		{
			blockedByPlannedLidarMask = controller.channels.tryEmitPlannedLidarMask(this, args);
		}
		
		struct PlannedLidarMask_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<BoolLidarMask> _args;
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
			
			void trigger(void* sender, std::tuple<BoolLidarMask> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} plannedLidarMask_in;
		
		bool canReceiveLidarMasks_ext(std::tuple<std::vector<BoolLidarMask>> args)
		{
			if(blockedByLidarMasks_ext != nullptr)
				if(blockedByLidarMasks_ext->getSender() == this)
					return false;
												
			blockedByLidarMasks_ext = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByLidarMasks_ext = nullptr;
															
		inline void tryEmitLidarMasks_ext(std::tuple<std::vector<BoolLidarMask>> args)
		{
			blockedByLidarMasks_ext = controller.channels.tryEmitLidarMasks_ext(this, args);
		}
		
		struct LidarMasks_ext_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<BoolLidarMask>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<BoolLidarMask>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} lidarMasks_ext_in;
		
		bool canReceivePlannedLidarMask_ext(std::tuple<BoolLidarMask> args)
		{
			if(blockedByPlannedLidarMask_ext != nullptr)
				if(blockedByPlannedLidarMask_ext->getSender() == this)
					return false;
												
			blockedByPlannedLidarMask_ext = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByPlannedLidarMask_ext = nullptr;
															
		inline void tryEmitPlannedLidarMask_ext(std::tuple<BoolLidarMask> args)
		{
			blockedByPlannedLidarMask_ext = controller.channels.tryEmitPlannedLidarMask_ext(this, args);
		}
		
		struct PlannedLidarMask_ext_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<BoolLidarMask> _args;
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
			
			void trigger(void* sender, std::tuple<BoolLidarMask> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} plannedLidarMask_ext_in;
		
		bool canReceiveLidarMasks_Plan(std::tuple<std::vector<BoolLidarMask>> args)
		{
			if(blockedByLidarMasks_Plan != nullptr)
				if(blockedByLidarMasks_Plan->getSender() == this)
					return false;
												
			blockedByLidarMasks_Plan = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByLidarMasks_Plan = nullptr;
															
		inline void tryEmitLidarMasks_Plan(std::tuple<std::vector<BoolLidarMask>> args)
		{
			blockedByLidarMasks_Plan = controller.channels.tryEmitLidarMasks_Plan(this, args);
		}
		
		struct LidarMasks_Plan_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<BoolLidarMask>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<BoolLidarMask>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} lidarMasks_Plan_in;
		
		bool canReceivePlannedLidarMask_Plan(std::tuple<BoolLidarMask> args)
		{
			if(blockedByPlannedLidarMask_Plan != nullptr)
				if(blockedByPlannedLidarMask_Plan->getSender() == this)
					return false;
												
			blockedByPlannedLidarMask_Plan = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByPlannedLidarMask_Plan = nullptr;
															
		inline void tryEmitPlannedLidarMask_Plan(std::tuple<BoolLidarMask> args)
		{
			blockedByPlannedLidarMask_Plan = controller.channels.tryEmitPlannedLidarMask_Plan(this, args);
		}
		
		struct PlannedLidarMask_Plan_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<BoolLidarMask> _args;
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
			
			void trigger(void* sender, std::tuple<BoolLidarMask> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} plannedLidarMask_Plan_in;
		
		bool canReceiveLidarMasks_Legitimate(std::tuple<std::vector<BoolLidarMask>> args)
		{
			if(blockedByLidarMasks_Legitimate != nullptr)
				if(blockedByLidarMasks_Legitimate->getSender() == this)
					return false;
												
			blockedByLidarMasks_Legitimate = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByLidarMasks_Legitimate = nullptr;
															
		inline void tryEmitLidarMasks_Legitimate(std::tuple<std::vector<BoolLidarMask>> args)
		{
			blockedByLidarMasks_Legitimate = controller.channels.tryEmitLidarMasks_Legitimate(this, args);
		}
		
		struct LidarMasks_Legitimate_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<std::vector<BoolLidarMask>> _args;
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
			
			void trigger(void* sender, std::tuple<std::vector<BoolLidarMask>> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} lidarMasks_Legitimate_in;
		
		bool canReceivePlannedLidarMask_Legitimate(std::tuple<BoolLidarMask> args)
		{
			if(blockedByPlannedLidarMask_Legitimate != nullptr)
				if(blockedByPlannedLidarMask_Legitimate->getSender() == this)
					return false;
												
			blockedByPlannedLidarMask_Legitimate = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByPlannedLidarMask_Legitimate = nullptr;
															
		inline void tryEmitPlannedLidarMask_Legitimate(std::tuple<BoolLidarMask> args)
		{
			blockedByPlannedLidarMask_Legitimate = controller.channels.tryEmitPlannedLidarMask_Legitimate(this, args);
		}
		
		struct PlannedLidarMask_Legitimate_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<BoolLidarMask> _args;
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
			
			void trigger(void* sender, std::tuple<BoolLidarMask> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} plannedLidarMask_Legitimate_in;
		
		bool canReceiveDirections(std::tuple<SpinConfig> args)
		{
			if(blockedByDirections != nullptr)
				if(blockedByDirections->getSender() == this)
					return false;
												
			blockedByDirections = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByDirections = nullptr;
															
		inline void tryEmitDirections(std::tuple<SpinConfig> args)
		{
			blockedByDirections = controller.channels.tryEmitDirections(this, args);
		}
		
		struct Directions_t : public EventBuffer
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
		} directions_in;
		
		bool canReceiveDirections_ext(std::tuple<SpinConfig> args)
		{
			if(blockedByDirections_ext != nullptr)
				if(blockedByDirections_ext->getSender() == this)
					return false;
												
			blockedByDirections_ext = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByDirections_ext = nullptr;
															
		inline void tryEmitDirections_ext(std::tuple<SpinConfig> args)
		{
			blockedByDirections_ext = controller.channels.tryEmitDirections_ext(this, args);
		}
		
		struct Directions_ext_t : public EventBuffer
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
		} directions_ext_in;
		
		bool canReceiveDirections_Legitimate(std::tuple<SpinConfig> args)
		{
			if(blockedByDirections_Legitimate != nullptr)
				if(blockedByDirections_Legitimate->getSender() == this)
					return false;
												
			blockedByDirections_Legitimate = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByDirections_Legitimate = nullptr;
															
		inline void tryEmitDirections_Legitimate(std::tuple<SpinConfig> args)
		{
			blockedByDirections_Legitimate = controller.channels.tryEmitDirections_Legitimate(this, args);
		}
		
		struct Directions_Legitimate_t : public EventBuffer
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
		} directions_Legitimate_in;
		
		bool canReceiveDirections_Execute(std::tuple<SpinConfig> args)
		{
			if(blockedByDirections_Execute != nullptr)
				if(blockedByDirections_Execute->getSender() == this)
					return false;
												
			blockedByDirections_Execute = nullptr;
		
			switch(currentState)
			{
				case s_Knowledge:
				{
					
					return false;
				}
				default:
				{
					return false;
				}
			}
		}

		EventBuffer* blockedByDirections_Execute = nullptr;
															
		inline void tryEmitDirections_Execute(std::tuple<SpinConfig> args)
		{
			blockedByDirections_Execute = controller.channels.tryEmitDirections_Execute(this, args);
		}
		
		struct Directions_Execute_t : public EventBuffer
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
		} directions_Execute_in;
		
	
		void execute() override
		{
			while(tryTransitions());
		
			switch(currentState)
			{
			case s_Knowledge:
			{
				knowledge_State.during();
				break;
			}
				default:
					break;
			}
		}
	
		bool tryTransitions() override
		{
			switch(currentState)
			{
				case s_Knowledge:
				{
					if(get_laserScans_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_laserScans_in.reset();
						this->tryEmitLaserScans(std::tuple<std::vector<LaserScan>>{this->laserScans_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(set_laserScans_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						set_laserScans_in.reset();
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_laserScans_ext_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_laserScans_ext_in.reset();
						this->tryEmitLaserScans_ext(std::tuple<std::vector<LaserScan>>{this->laserScans_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_laserScans_Analyse_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_laserScans_Analyse_in.reset();
						this->tryEmitLaserScans_Analyse(std::tuple<std::vector<LaserScan>>{this->laserScans_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_laserScans_Plan_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_laserScans_Plan_in.reset();
						this->tryEmitLaserScans_Plan(std::tuple<std::vector<LaserScan>>{this->laserScans_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_laserScans_Legitimate_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_laserScans_Legitimate_in.reset();
						this->tryEmitLaserScans_Legitimate(std::tuple<std::vector<LaserScan>>{this->laserScans_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_lidarMasks_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_lidarMasks_in.reset();
						this->tryEmitLidarMasks(std::tuple<std::vector<BoolLidarMask>>{this->lidarMasks_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(set_lidarMasks_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						set_lidarMasks_in.reset();
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_lidarMasks_ext_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_lidarMasks_ext_in.reset();
						this->tryEmitLidarMasks_ext(std::tuple<std::vector<BoolLidarMask>>{this->lidarMasks_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_plannedLidarMask_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_plannedLidarMask_in.reset();
						this->tryEmitPlannedLidarMask(std::tuple<BoolLidarMask>{this->plannedLidarMask_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(set_plannedLidarMask_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						set_plannedLidarMask_in.reset();
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_plannedLidarMask_ext_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_plannedLidarMask_ext_in.reset();
						this->tryEmitPlannedLidarMask_ext(std::tuple<BoolLidarMask>{this->plannedLidarMask_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_lidarMasks_Plan_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_lidarMasks_Plan_in.reset();
						this->tryEmitLidarMasks_Plan(std::tuple<std::vector<BoolLidarMask>>{this->lidarMasks_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_plannedLidarMask_Plan_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_plannedLidarMask_Plan_in.reset();
						this->tryEmitPlannedLidarMask_Plan(std::tuple<BoolLidarMask>{this->plannedLidarMask_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_lidarMasks_Legitimate_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_lidarMasks_Legitimate_in.reset();
						this->tryEmitLidarMasks_Legitimate(std::tuple<std::vector<BoolLidarMask>>{this->lidarMasks_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_plannedLidarMask_Legitimate_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_plannedLidarMask_Legitimate_in.reset();
						this->tryEmitPlannedLidarMask_Legitimate(std::tuple<BoolLidarMask>{this->plannedLidarMask_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_directions_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_directions_in.reset();
						this->tryEmitDirections(std::tuple<SpinConfig>{this->directions_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(set_directions_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						set_directions_in.reset();
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_directions_ext_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_directions_ext_in.reset();
						this->tryEmitDirections_ext(std::tuple<SpinConfig>{this->directions_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_directions_Legitimate_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_directions_Legitimate_in.reset();
						this->tryEmitDirections_Legitimate(std::tuple<SpinConfig>{this->directions_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}
					if(get_directions_Execute_in.getSender() != nullptr)
					{
						knowledge_State.exit();
						get_directions_Execute_in.reset();
						this->tryEmitDirections_Execute(std::tuple<SpinConfig>{this->directions_var});
						knowledge_State.enter();
						currentState = s_Knowledge;
						return true;	
					}

					break;
				}
				case j_i0:
				{
					
					;
					currentState = s_Knowledge;
					knowledge_State.enter();
					return true;
				}
				default:
					break;
			}
				
			return false;
		}
};
	
#endif
