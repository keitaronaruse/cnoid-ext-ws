/*
*/
#include <cnoid/Plugin>
#include <cnoid/MessageView>
#include <cnoid/ConnectionSet>
#include <cnoid/ItemList>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/TimeBar>
#include <cnoid/EigenTypes>
#include <vector>

class DevGuidePlugin : public cnoid::Plugin
{
    cnoid::ScopedConnectionSet connections;
    cnoid::ItemList<cnoid::BodyItem> bodyItems;
    std::vector<cnoid::Matrix3> initialRotations;
public:
    DevGuidePlugin() : Plugin("DevGuide")
    {
        require("Body");
    }

    virtual bool initialize() override
    {
        connections.add(
            cnoid::RootItem::instance()->sigSelectedItemsChanged().connect(
                [this](const cnoid::ItemList<>& selectedItems){
                    onSelectedItemsChanged(selectedItems);
                }));

        connections.add(
            cnoid::TimeBar::instance()->sigTimeChanged().connect(
                [this](double time){
                    return onTimeChanged(time);
                }));

        return(true);
    }

    void onSelectedItemsChanged(cnoid::ItemList<cnoid::BodyItem> selectedBodyItems)
    {
        if(selectedBodyItems != bodyItems){
            bodyItems = selectedBodyItems;
            initialRotations.clear();
            for(auto& bodyItem : bodyItems){
                cnoid::Body* body = bodyItem->body();
                cnoid::Link* rootLink = body->rootLink();
                initialRotations.push_back(rootLink->rotation());
            }
        }
    }

    bool onTimeChanged(double time)
    {
        for(size_t i=0; i < bodyItems.size(); ++i){
            auto bodyItem = bodyItems[i];
            cnoid::Matrix3 R = cnoid::AngleAxis(time, cnoid::Vector3::UnitZ()) * initialRotations[i];
            bodyItem->body()->rootLink()->setRotation(R);
            bodyItem->notifyKinematicStateChange(true);
        }

        return !bodyItems.empty();
    }

    virtual bool finalize() override
    {
        return(true);
    }
    virtual const char* description() const override
    {
        return("DevGuidePlugin by K. Naruse");
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(DevGuidePlugin)
