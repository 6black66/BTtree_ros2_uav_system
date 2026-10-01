#include <behaviortree_cpp/bt_factory.h>

#include <exception>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
  if(argc < 3)
  {
    std::cerr << "expected <tree> <plugin> [plugin ...]\n";
    return 2;
  }

  try
  {
    BT::BehaviorTreeFactory factory;
    const std::string tree = argv[1];
    for(int index = 2; index < argc; ++index)
    {
      const std::string plugin = argv[index];
      factory.registerFromPlugin(plugin);
      std::cout << "loaded " << plugin << '\n';
    }
    factory.registerBehaviorTreeFromFile(tree);
    std::cout << "parsed " << tree << '\n';

    if(factory.registeredBehaviorTrees().size() != 1U)
    {
      std::cerr << "not all phase trees were registered\n";
      return 3;
    }
  }
  catch(const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
