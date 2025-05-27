import React from 'react';
import { CheckCircle2, BarChart2, Zap, ShieldCheck, Cpu, Users } from 'lucide-react';

const WhyChooseUs: React.FC = () => {
  const reasons = [
    {
      icon: <Cpu className="h-6 w-6 text-green-500" />,
      title: "Cutting-edge Technology",
      description: "Our robots utilize the latest advancements in AI, machine learning, and autonomous navigation."
    },
    {
      icon: <Users className="h-6 w-6 text-green-500" />,
      title: "Expert Team",
      description: "Our team consists of leading robotics engineers and AI specialists with decades of combined experience."
    },
    {
      icon: <BarChart2 className="h-6 w-6 text-green-500" />,
      title: "Proven Results",
      description: "We have a track record of successful implementations across educational and industrial settings."
    },
    {
      icon: <Zap className="h-6 w-6 text-green-500" />,
      title: "Efficiency Focus",
      description: "Our solutions are designed to maximize productivity and efficiency in every application."
    },
    {
      icon: <ShieldCheck className="h-6 w-6 text-green-500" />,
      title: "Safety First",
      description: "All our robotics solutions prioritize safety with multiple redundant systems."
    },
    {
      icon: <CheckCircle2 className="h-6 w-6 text-green-500" />,
      title: "Educational Commitment",
      description: "We're dedicated to fostering the next generation of robotics engineers through our educational programs."
    }
  ];

  return (
    <section className="py-20 bg-black text-white">
      <div className="container mx-auto px-4">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-5xl font-bold">
            Why Choose <span className="text-green-500">KKR</span>
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mt-6"></div>
        </div>
        
        <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8">
          {reasons.map((reason, index) => (
            <div 
              key={index}
              className="p-6 rounded-xl border border-gray-800 hover:border-green-500 transition-all duration-300"
            >
              <div className="flex items-start mb-4">
                <div className="bg-green-500/20 p-3 rounded-md mr-4">
                  {reason.icon}
                </div>
                <h3 className="text-xl font-bold">{reason.title}</h3>
              </div>
              <p className="text-gray-400 pl-16">{reason.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default WhyChooseUs;