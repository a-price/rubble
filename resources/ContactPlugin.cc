// Adapted from the Contact Plugin at http://gazebosim.org/wiki/Tutorials/1.9/sensors/contact

#include "ContactPlugin.hh"

#include "gazebo_msgs/ContactsState.h"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
	contactPub = nh.advertise<gazebo_msgs::ContactsState>("/gazebo/contacts", 1);
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

  // TODO: Don't publish duplicate states
  if (contacts.contact_size() == 0)
  {
    return; //This shouldn't happen. :( 
  }
  gazebo_msgs::ContactsState cs;
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
	  if ("ground_plane::link::collision" == contacts.contact(i).collision1() ||
		  "ground_plane::link::collision" == contacts.contact(i).collision2())
	  {
		  continue;
	  }

	  gazebo_msgs::ContactState c;
	  c.collision1_name = contacts.contact(i).collision1();
	  c.collision2_name = contacts.contact(i).collision2();

	  for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
	  {
		  geometry_msgs::Vector3 pos;
		  geometry_msgs::Vector3 norm;

		  pos.x = contacts.contact(i).position(j).x();
		  pos.y = contacts.contact(i).position(j).y();
		  pos.z = contacts.contact(i).position(j).z();

		  norm.x = contacts.contact(i).normal(j).x();
		  norm.y = contacts.contact(i).normal(j).y();
		  norm.z = contacts.contact(i).normal(j).z();

		  geometry_msgs::Wrench w;
		  // Throw out incidental contacts
		  if (fabs(contacts.contact(i).wrench(j).body_1_wrench().force().z()) +
			  fabs(contacts.contact(i).wrench(j).body_2_wrench().force().z()) < 0.1)
		  {
			  continue;
		  }

		  // Find which wrench points up; this ought to be the upper item
		  if (contacts.contact(i).wrench(j).body_1_wrench().force().z() >
			  contacts.contact(i).wrench(j).body_2_wrench().force().z())
		  {
			  c.info = c.collision1_name;
		  }
		  else
		  {
			  c.info = c.collision2_name;
		  }

		  // TODO: Add wrench to message if necessary

		  c.contact_positions.push_back(pos);
		  c.contact_normals.push_back(norm);
		  c.depths.push_back(contacts.contact(i).depth(j));
	  }

	  cs.states.push_back(c);
  }

  cs.header.stamp = ros::Time::now();
  contactPub.publish(cs);
}
