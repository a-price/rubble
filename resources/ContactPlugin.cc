// Adapted from the Contact Plugin at http://gazebosim.org/wiki/Tutorials/1.9/sensors/contact

#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{ 
}   

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{   
  // Get the parent sensor.
  this->parentSensor =
    boost::shared_dynamic_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  { 
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  } 

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();
  std::vector<std::pair<std::string, std::string> >contacted; // declaring vector of pairs of contacts
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
     contacted.push_back(make_pair(contacts.contact(i).collision1(), contacts.contact(i).collision2()));


   /* for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
    }*/
  }

  for(std::vector<std::pair<std::string, std::string> >::iterator it = contacted.begin(); it!=contacted.end(); it++)
  {
    std:: cout << "<" << it->first << ", " << it->second << ")" << std::endl;
  }
}
