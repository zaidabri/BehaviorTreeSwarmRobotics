#include <iostream>
#include <chrono>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

using namespace std::chrono_literals;

//Battery subtree
BT::NodeStatus BatteryLessThan20()
{
  std::cout << "Battery OK" << std::endl;
  return BT::NodeStatus::SUCCESS;
}
// If we got a FAILURE on the last node the robot will go to the charging station
class GoChargingStation : public BT::SyncActionNode
{
public:
  explicit GoChargingStation(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    //We give delay of five seconds to simulate the robot going to the charging station
    std::this_thread::sleep_for(5s);
    std::cout << "Charging" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
//Condition node to check it there is an order available
BT::NodeStatus OrderAvailable()
{
  std::cout << "OrderAvailable" << std::endl;
  return BT::NodeStatus::SUCCESS;
}
//Get the number of the pickup station where to pick the order
class GetOrderStation : public BT::SyncActionNode
{
public:
  explicit GetOrderStation(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("OrderStation") };
  }

  BT::NodeStatus tick() override
  {

    std::this_thread::sleep_for(5s);
    setOutput("OrderStation", "2" );
    std::cout << "Getting information of the pickup station..." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
//Read from Blackboard the {PickupStation} to get the station to pick the order
class ReserveOrder : public BT::SyncActionNode
{
public:
  ReserveOrder(const std::string& name, const BT::NodeConfiguration& config)
    : SyncActionNode(name, config)
  { }

  static BT::PortsList providedPorts()
  {
    // This action has a single input port called "message"
    return { BT::InputPort<std::string>("PickupStation") };
  }

  BT::NodeStatus tick() override
  {
    BT::Optional<std::string> msg = getInput<std::string>("PickupStation");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [PickupStation]: ", 
                              msg.error() );
    }
    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }  
};
/*After the ordr is reserved the robot looks for the position of the pickupstation
then it send a signal */
class FindPickupStation : public BT::SyncActionNode
{
public:
  explicit FindPickupStation(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    //We give delay of five seconds to simulate the robot going to the charging station
    std::this_thread::sleep_for(5s);
    std::cout << "Pickup station found!" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
//Does the robot find a obstacle?
BT::NodeStatus ObstacleFound()
{
  std::cout << "Obstacle found..." << std::endl;
  return BT::NodeStatus::SUCCESS;
}
//If there is an obstacle the robot will recalculate the route 
class RecalculateRoute : public BT::SyncActionNode
{
public:
  explicit RecalculateRoute(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    //We give delay of five seconds to simulate the robot going to the charging station
    std::this_thread::sleep_for(5s);
    std::cout << "Route recalculated!" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
//Robot follows the route calculated
class MoveForward : public BT::SyncActionNode
{
public:
  explicit MoveForward(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    //We give delay of five seconds to simulate the robot going to the charging station
    std::this_thread::sleep_for(5s);
    std::cout << "Moving!" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
//Robot picks the order
class PickOrder : public BT::SyncActionNode
{
public:
  explicit PickOrder(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    //We give delay of five seconds to simulate the robot going to the charging station
    std::cout << "Picking order" << std::endl;
    std::this_thread::sleep_for(7s);
    std::cout << "Order picked" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
//The robot choses a free transfer station
class FindTransferStation : public BT::SyncActionNode
{
public:
  explicit FindTransferStation(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(7s);
    std::cout << "Transfer station found!" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
/*The robot sends a signal to inform the robots in the other area 
in which transfer station it will deliver the package */
class SendDeliverySignal : public BT::SyncActionNode
{
public:
  explicit SendDeliverySignal(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("ChosenTransferStation") };
  }


  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(5s);
    setOutput("ChosenTransferStation", "Transfer station chosen: 4" );
    std::cout << "Transfer station chosen: 4" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
//The robot delivers the order 
class DeliverOrder : public BT::SyncActionNode
{
public:
  explicit DeliverOrder(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(7s);
    std::cout << "Order delivered!" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
//The robot set the status as free
class SetStatusAsFree : public BT::SyncActionNode
{
public:
  explicit SetStatusAsFree(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(7s);
    std::cout << "Free!" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};


 

int main()
{
  BT::BehaviorTreeFactory factory;

  //register actions
  factory.registerNodeType<GoChargingStation>("GoChargingStation");
  factory.registerNodeType<ReserveOrder>("ReserveOrder");
  factory.registerNodeType<FindPickupStation>("FindPickupStation");
  factory.registerNodeType<RecalculateRoute>("RecalculateRoute");
  factory.registerNodeType<MoveForward>("MoveForward");
  factory.registerNodeType<PickOrder>("PickOrder");
  factory.registerNodeType<FindTransferStation>("FindTransferStation");
  factory.registerNodeType<SendDeliverySignal>("SendDeliverySignal");
  factory.registerNodeType<DeliverOrder>("DeliverOrder");
  factory.registerNodeType<SetStatusAsFree>("SetStatusAsFree");
  factory.registerNodeType<GetOrderStation>("GetOrderStation");
  
  //register condition
  factory.registerSimpleCondition("BatteryLessThan20", std::bind(BatteryLessThan20));
  factory.registerSimpleCondition("OrderAvailable", std::bind(OrderAvailable));
  factory.registerSimpleCondition("ObstacleFound", std::bind(ObstacleFound));
  
  
  //Create Tree
  auto tree = factory.createTreeFromFile("/Users/zaidabrito/BTree/BTreeWork/SwarmBTDemo/finaltree.xml");

  //execute the tree
  auto status = tree.tickRoot();
  while (status == BT::NodeStatus::SUCCESS) {
    status = tree.tickRoot();
  }

  return 0;
}
