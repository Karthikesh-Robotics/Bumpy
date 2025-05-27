import React, { useEffect, useState } from 'react';
import { useParams, useNavigate } from 'react-router-dom';
import { Calendar, MapPin, ArrowLeft } from 'lucide-react';
import { achievements } from '../data/achievements';

const WorkshopDetail: React.FC = () => {
  const { id } = useParams();
  const navigate = useNavigate();
  const [currentImageIndex, setCurrentImageIndex] = useState(0);

  const workshop = achievements.find(a => a.id === id);

  useEffect(() => {
    if (!workshop) {
      navigate('/achievements');
    }
  }, [workshop, navigate]);

  if (!workshop) return null;

  const topics = [
    'Introduction to Core Concepts',
    'Practical Implementation',
    'Hands-on Exercises',
    'Real-world Applications',
    'Project Development',
  ];

  const activities = [
    'Interactive Demonstrations',
    'Group Projects',
    'Problem-solving Sessions',
    'Live Coding',
    'Q&A Sessions',
  ];

  return (
    <div className="min-h-screen bg-gray-50 py-12">
      <div className="container mx-auto px-4">
        <button
          onClick={() => navigate('/achievements')}
          className="flex items-center gap-2 text-gray-600 hover:text-green-500 transition-colors mb-8 group"
        >
          <ArrowLeft className="h-5 w-5 transform group-hover:-translate-x-1 transition-transform" />
          Back to Achievements
        </button>

        <div className="bg-white rounded-2xl shadow-xl overflow-hidden">
          <div className="relative h-96">
            <img
              src={workshop.images[currentImageIndex]}
              alt={workshop.title}
              className="w-full h-full object-cover"
            />
            <div className="absolute inset-0 bg-gradient-to-t from-black/50 to-transparent" />
            <div className="absolute bottom-0 left-0 right-0 p-8 text-white">
              <h1 className="text-4xl font-bold mb-4">{workshop.title}</h1>
              <div className="flex items-center gap-6">
                <div className="flex items-center gap-2">
                  <Calendar className="h-5 w-5" />
                  <span>{workshop.date}</span>
                </div>
                <div className="flex items-center gap-2">
                  <MapPin className="h-5 w-5" />
                  <span>{workshop.location}</span>
                </div>
              </div>
            </div>
          </div>

          <div className="p-8">
            <div className="grid md:grid-cols-2 gap-12">
              <div className="space-y-8 animate-slideInLeft">
                <section>
                  <h2 className="text-2xl font-semibold mb-4">Topics Covered</h2>
                  <div className="space-y-3">
                    {topics.map((topic, index) => (
                      <div
                        key={index}
                        className="flex items-center gap-3 p-3 bg-gray-50 rounded-lg transform hover:scale-105 transition-transform"
                      >
                        <div className="h-2 w-2 rounded-full bg-green-500" />
                        <span>{topic}</span>
                      </div>
                    ))}
                  </div>
                </section>

                <section>
                  <h2 className="text-2xl font-semibold mb-4">Engaging Activities</h2>
                  <div className="space-y-3">
                    {activities.map((activity, index) => (
                      <div
                        key={index}
                        className="flex items-center gap-3 p-3 bg-gray-50 rounded-lg transform hover:scale-105 transition-transform"
                      >
                        <div className="h-2 w-2 rounded-full bg-green-500" />
                        <span>{activity}</span>
                      </div>
                    ))}
                  </div>
                </section>
              </div>

              <div className="space-y-6 animate-slideInRight">
                <h2 className="text-2xl font-semibold mb-4">Photo Gallery</h2>
                <div className="grid grid-cols-2 gap-4">
                  {workshop.images.map((image, index) => (
                    <div
                      key={index}
                      className={`rounded-lg overflow-hidden cursor-pointer transform hover:scale-105 transition-transform ${
                        index === 0 ? 'col-span-2 row-span-2' : ''
                      }`}
                      onClick={() => setCurrentImageIndex(index)}
                    >
                      <img
                        src={image}
                        alt={`Workshop photo ${index + 1}`}
                        className="w-full h-full object-cover"
                      />
                    </div>
                  ))}
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default WorkshopDetail;