import React from 'react';
import { Briefcase, Lightbulb, Award, ArrowRight } from 'lucide-react';

const Internship: React.FC = () => {
  return (
    <section id="internship" className="py-20 bg-gray-50">
      <div className="container mx-auto px-4">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-4xl font-bold mb-4">
            Join Our <span className="text-green-500">Internship</span> Program
          </h2>
          <div className="w-24 h-1 bg-green-500 mx-auto mb-6"></div>
          <p className="text-gray-600 max-w-2xl mx-auto">
            Kickstart your career in robotics and gain hands-on experience with cutting-edge technology
          </p>
        </div>

        <div className="max-w-7xl mx-auto">
          <div className="grid grid-cols-1 lg:grid-cols-12 gap-8 items-center">
            {/* Left side - Image */}
            <div className="lg:col-span-5 overflow-hidden rounded-xl shadow-lg">
              <img 
                src="https://images.pexels.com/photos/3861958/pexels-photo-3861958.jpeg" 
                alt="Robotics Internship" 
                className="w-full h-full object-cover transition-transform duration-700 hover:scale-110"
              />
            </div>
            
            {/* Right side - Content */}
            <div className="lg:col-span-7 space-y-8">
              <h3 className="text-2xl font-bold text-gray-800">
                Want to be a part of <span className="text-green-500">KKR Internship</span>?
              </h3>
              
              <p className="text-gray-600">
                Our internship program offers students and recent graduates the opportunity to work alongside industry experts, 
                develop valuable skills, and contribute to innovative robotics projects. Interns gain practical experience in 
                robotics development, computer vision, machine learning, and more.
              </p>
              
              <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
                <div className="bg-white p-5 rounded-lg shadow-md transition-all duration-300 hover:shadow-xl hover:-translate-y-1">
                  <Briefcase className="h-10 w-10 text-green-500 mb-3" />
                  <h4 className="font-semibold text-lg mb-2">Hands-on Experience</h4>
                  <p className="text-gray-600 text-sm">Work on real-world projects with cutting-edge technology</p>
                </div>
                
                <div className="bg-white p-5 rounded-lg shadow-md transition-all duration-300 hover:shadow-xl hover:-translate-y-1">
                  <Lightbulb className="h-10 w-10 text-green-500 mb-3" />
                  <h4 className="font-semibold text-lg mb-2">Expert Mentorship</h4>
                  <p className="text-gray-600 text-sm">Learn from industry professionals and receive guidance</p>
                </div>
                
                <div className="bg-white p-5 rounded-lg shadow-md transition-all duration-300 hover:shadow-xl hover:-translate-y-1">
                  <Award className="h-10 w-10 text-green-500 mb-3" />
                  <h4 className="font-semibold text-lg mb-2">Career Growth</h4>
                  <p className="text-gray-600 text-sm">Build your portfolio and expand your professional network</p>
                </div>
              </div>
              
              <div className="bg-gray-100 p-6 rounded-lg border-l-4 border-green-500">
                <p className="text-gray-700 font-medium">
                  Applications are now open for our Summer 2025 internship cohort. Limited spots available!
                </p>
              </div>
              
              <a 
                href="https://forms.gle/tnDsuXXk42d5SzpE6" 
                target="_blank" 
                rel="noopener noreferrer" 
                className="inline-block bg-gray-900 hover:bg-green-500 text-white font-semibold py-4 px-8 rounded-md transition-all duration-300 flex items-center gap-3 group"
              >
                Apply Now
                <ArrowRight className="h-5 w-5 transform group-hover:translate-x-1 transition-transform" />
              </a>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default Internship;