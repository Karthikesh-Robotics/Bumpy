import React from 'react';
import { Heart, Award, Users, BookOpen } from 'lucide-react';

const Values: React.FC = () => {
  const values = [
    {
      icon: <Heart className="h-8 w-8 text-green-500" />,
      title: "Technology for Everyone",
      description: "We believe that technology should be accessible to all, regardless of their financial circumstances or background."
    },
    {
      icon: <Award className="h-8 w-8 text-green-500" />,
      title: "Excellence",
      description: "We strive for excellence in everything we do, from our products to our customer service and educational outreach."
    },
    {
      icon: <Users className="h-8 w-8 text-green-500" />,
      title: "Empowerment",
      description: "By providing affordable, magnificent technology education to students, we aim to empower the succeeding generation of innovators."
    },
    {
      icon: <BookOpen className="h-8 w-8 text-green-500" />,
      title: "Knowledge Sharing",
      description: "We are committed to democratizing technology and creating a more equitable future for all through education."
    }
  ];

  return (
    <section className="py-20 bg-gray-50">
      <div className="container mx-auto px-4">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-5xl font-bold text-black">
            Our Core Values
          </h2>
          <p className="text-gray-600 mt-4 max-w-2xl mx-auto">
            At our startup, "Technology for everyone" is not just a key formula, it's our guiding tenet.
          </p>
          <div className="w-24 h-1 bg-green-500 mx-auto mt-6"></div>
        </div>
        
        <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-8">
          {values.map((value, index) => (
            <div 
              key={index} 
              className="bg-white p-8 rounded-xl shadow-lg transform transition-all duration-300 hover:shadow-xl hover:-translate-y-2"
            >
              <div className="bg-black p-4 rounded-full inline-flex mb-6">
                {value.icon}
              </div>
              <h3 className="text-xl font-bold mb-4">{value.title}</h3>
              <p className="text-gray-600">{value.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default Values;