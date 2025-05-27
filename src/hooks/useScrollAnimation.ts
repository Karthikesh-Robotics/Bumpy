import { useEffect } from 'react';

export const useScrollAnimation = () => {
  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          const target = entry.target as HTMLElement;
          if (entry.isIntersecting) {
            target.style.opacity = '1';
            target.style.transform = 'translateY(0)';
            target.style.transition = 'opacity 0.6s ease-out, transform 0.6s ease-out';

            // Optional: unobserve after animation
            observer.unobserve(entry.target);
          }
        });
      },
      {
        threshold: 0.3,
      }
    );

    const sections = document.querySelectorAll('.scroll-section');
    sections.forEach((section) => {
      const el = section as HTMLElement;
      // Initial state for animation
      el.style.opacity = '0';
      el.style.transform = 'translateY(30px)';
      observer.observe(el);
    });

    return () => {
      sections.forEach((section) => observer.unobserve(section));
    };
  }, []);
};
