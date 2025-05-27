import React from 'react';
import { Target, Eye } from 'lucide-react';

const MissionVision: React.FC = () => {
  return (
    <section className="py-20 bg-white">
      <div className="container mx-auto px-4">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-5xl font-bold text-black">
            Our Mission & Vision
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mt-6"></div>
        </div>
        
        <div className="grid md:grid-cols-2 gap-12">
          {/* Mission */}
          <div className="bg-black text-white p-8 rounded-xl transform transition-transform duration-500 hover:scale-105 hover:-rotate-1">
            <div className="flex items-center mb-6">
              <div className="bg-green-500 p-3 rounded-full mr-4">
                <Target className="h-8 w-8 text-black" />
              </div>
              <h3 className="text-2xl md:text-3xl font-bold">Our Mission</h3>
            </div>
            
            <p className="text-gray-300 leading-relaxed">
              Our mission is to revolutionize homes and industries empowered by trailblazing robotic technology 
              that enhance and automate everyday errands with ameliorated safety, productivity and ease of use. 
              We cater to the requirements of our customers by providing unparalleled services, in the sector of 
              Robotics and Automation. We extend our phenomenal services to commoners within affordable prices. 
              We also intend to impart knowledge to the budding young minds.
            </p>
          </div>
          
          {/* Vision */}
          <div className="bg-green-500 text-black p-8 rounded-xl transform transition-transform duration-500 hover:scale-105 hover:rotate-1">
            <div className="flex items-center mb-6">
              <div className="bg-black p-3 rounded-full mr-4">
                <Eye className="h-8 w-8 text-green-500" />
              </div>
              <h3 className="text-2xl md:text-3xl font-bold">Our Vision</h3>
            </div>
            
            <p className="text-black leading-relaxed">
              We envision to engineer robotic solutions that are user-friendly, safe and economical. 
              Our vision is to elevate the standard of living and to boost industries with unprecedented 
              levels of efficiency. By overstepping the frontiers of the possibilities, we strive to 
              construct a better future for ourselves and for tomorrow's citizens.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
};

export default MissionVision;