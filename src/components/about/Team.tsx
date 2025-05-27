import React, { useState } from 'react';
import { Linkedin, Twitter, Mail } from 'lucide-react';

interface TeamMember {
  name: string;
  position: string;
  image: string;
  socials: {
    linkedin?: string;
    twitter?: string;
    email?: string;
  };
}

const Team: React.FC = () => {
  const teamMembers: TeamMember[] = [
    {
      name: "Karthikesh J G",
      position: "CEO & Founder",
      image: "/assets/team/1.jpg",
      socials: {
        linkedin: "https://www.linkedin.com/in/karthikeshjg/",
        twitter: "#",
        email: "mailto:ceo@karthikeshrobotics.in"
      }
    },
    {
      name: "Veera Saravanan S",
      position: "COO",
      image: "/assets/team/2.jpg",
      socials: {
        linkedin: "https://www.linkedin.com/in/veerasaravanans/",
        twitter: "#",
        email: "mailto:coo@karthikeshrobotics.in"
      }
    },
    {
      name: "Rishi",
      position: "Chief Development Officer",
      image: "/assets/team/3.jpg",
      socials: {
        linkedin: "#",
        twitter: "#",
        email: "mailto:cdo@karthikeshrobotics.in"
      }
    },
    {
      name: "Janisha G",
      position: "CDO",
      image: "/assets/team/4.jpg",
      socials: {
        linkedin: "https://www.linkedin.com/in/janisha-gopi-b01434257/",
        twitter: "#",
        email: "mailto:cdo2@karthikeshrobotics.in"
      }
    },
    {
      name: "Abish",
      position: "Chief AI Engineer",
      image: "/assets/team/5.jpg",
      socials: {
        linkedin: "https://www.linkedin.com/in/m-abish-9b8881252/",
        email: "mailto:ai@karthikeshrobotics.in"
      }
    },
    {
      name: "Abdul Rasheed",
      position: "CTO",
      image: "/assets/team/6.jpg",
      socials: {
        linkedin: "https://www.linkedin.com/in/abdul-rasheed-06b46133b",
        twitter: "#",
        email: "mailto:cto@karthikeshrobotics.in"
      }
    },
    {
      name: "Vajreswari",
      position: "HR Manager",
      image: "/assets/team/7.jpg",
      socials: {
        linkedin: "https://www.linkedin.com/in/vajreswarisk/",
        email: "mailto:hr@karthikeshrobotics.in"
      }
    },
    {
      name: "Dhushyanth",
      position: "Chief Robotics Engineer",
      image: "/assets/team/8.jpg",
      socials: {
        linkedin: "#",
        twitter: "#",
        email: "mailto:robotics@karthikeshrobotics.in"
      }
    },
    {
      name: "Pranesh",
      position: "Senior Robotics Engineer",
      image: "/assets/team/9.jpg",
      socials: {
        linkedin: "#",
        twitter: "#",
        email: "mailto:robotics2@karthikeshrobotics.in"
      }
    }
  ];

  const [activeTab, setActiveTab] = useState('leadership');

  return (
    <section className="py-20 bg-white">
      <div className="container mx-auto px-4">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-5xl font-bold text-black">
            Meet Our Team
          </h2>
          <p className="text-gray-600 mt-4 max-w-2xl mx-auto">
            Our exceptional team of experts brings together decades of experience in robotics, AI, and business leadership.
          </p>
          <div className="w-24 h-1 bg-green-500 mx-auto mt-6"></div>
        </div>

        <div className="flex justify-center mb-12">
          <button 
            className={`px-6 py-2 mr-4 rounded-full transition-all duration-300 ${
              activeTab === 'leadership' ? 'bg-green-500 text-white' : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
            }`}
            onClick={() => setActiveTab('leadership')}
          >
            Leadership Team
          </button>
        </div>
        
        <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8">
          {teamMembers.map((member, index) => (
            <div 
              key={index}
              className="group relative overflow-hidden rounded-xl shadow-lg"
            >
              <div className="aspect-[3/4] overflow-hidden">
                <img 
                  src={member.image} 
                  alt={member.name}
                  className="w-full h-full object-cover object-center transition-transform duration-500 group-hover:scale-110"
                />
                <div className="absolute inset-0 bg-gradient-to-t from-black/80 via-black/50 to-transparent"></div>
              </div>
              <div className="absolute bottom-0 left-0 w-full p-6 transition-all duration-300 group-hover:bottom-4">
                <h3 className="text-xl font-bold text-white">{member.name}</h3>
                <p className="text-green-400 mb-4">{member.position}</p>
                <div className="flex space-x-3 opacity-0 -translate-y-4 transition-all duration-300 group-hover:opacity-100 group-hover:translate-y-0">
                  {member.socials.linkedin && (
                    <a href={member.socials.linkedin} className="bg-white p-2 rounded-full hover:bg-green-500 transition-colors duration-300" target="_blank" rel="noopener noreferrer">
                      <Linkedin className="h-5 w-5 text-black" />
                    </a>
                  )}
                  {member.socials.twitter && (
                    <a href={member.socials.twitter} className="bg-white p-2 rounded-full hover:bg-green-500 transition-colors duration-300">
                      <Twitter className="h-5 w-5 text-black" />
                    </a>
                  )}
                  {member.socials.email && (
                    <a href={member.socials.email} className="bg-white p-2 rounded-full hover:bg-green-500 transition-colors duration-300">
                      <Mail className="h-5 w-5 text-black" />
                    </a>
                  )}
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default Team;
