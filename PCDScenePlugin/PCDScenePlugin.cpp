/*
    PCDScenePlugin.cpp
        Display a PCD file
        Author: Keitaro Naruse
        Date:   2021-08-11
        MIT License
*/
#include <cnoid/Plugin>
#include <cnoid/MessageView>

class PCDScenePlugin : public cnoid::Plugin
{
public:
    PCDScenePlugin() : Plugin("PCDScene")
    {
        ;
    }

    virtual bool initialize() override
    {
        cnoid::MessageView::instance()->putln("PCDScenePlugin::Initialted.");
        return true;
    }

    virtual bool finalize() override
    {
        return(true);
    }
    virtual const char* description() const override
    {
        return("PCD Scene Plugin by K. Naruse");
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(PCDScenePlugin)
