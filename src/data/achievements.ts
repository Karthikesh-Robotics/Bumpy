export interface Achievement {
  id: string;
  date: string;
  title: string;
  location: string;
  category: string;
  images: string[];
  year: number;
}

// Helper function to generate image paths for each workshop
const getLocalWorkshopImages = (workshopId: number, count: number = 4) => {
  return Array.from({ length: count }, (_, i) =>
    `/assets/workshops/workshop${workshopId}/w${i + 1}.jpg`
  );
};

export const achievements: Achievement[] = [
  {
    id: "1",
    date: "7th-8th Nov 2024",
    title: "RoboDK & ROS Workshop",
    location: "Saveetha Engineering College, Chennai",
    category: "RoboDK & ROS",
    images: getLocalWorkshopImages(1),
    year: 2024
  },
  {
    id: "2",
    date: "25th-27th Nov 2024",
    title: "ROS2 Realms: A Hands-On Odyssey",
    location: "CIT, Chennai",
    category: "ROS",
    images: getLocalWorkshopImages(2),
    year: 2024
  },
  {
    id: "3",
    date: "3rd Feb 2025",
    title: "AR VR Workshop",
    location: "CIT, Chennai",
    category: "AR VR",
    images: getLocalWorkshopImages(3),
    year: 2025
  },
  {
    id: "4",
    date: "8th-12th Feb 2025",
    title: "ROS 5-Day Intensive Workshop",
    location: "KPR, Coimbatore",
    category: "ROS",
    images: getLocalWorkshopImages(4),
    year: 2025
  },
  {
    id: "5",
    date: "13th-15th Feb 2025",
    title: "ROS2 3-Day Workshop",
    location: "Ramakrishna Engineering College",
    category: "ROS",
    images: getLocalWorkshopImages(5),
    year: 2025
  },
  {
    id: "6",
    date: "17th-19th Feb 2025",
    title: "ROS2 3-Day Workshop",
    location: "Ramakrishna Engineering College",
    category: "ROS",
    images: getLocalWorkshopImages(6),
    year: 2025
  },
  {
    id: "7",
    date: "22nd Mar 2025",
    title: "ROS2 Workshop",
    location: "PSG College of Technology",
    category: "ROS",
    images: getLocalWorkshopImages(7),
    year: 2025
  }
];
