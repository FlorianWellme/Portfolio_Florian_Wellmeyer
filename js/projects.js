// js/projects.js

document.addEventListener("DOMContentLoaded", () => {
  const projectItems = document.querySelectorAll(".project-item");
  const container = document.getElementById("projectDetailsContainer");

  // Détails pour chaque projet
  const projectDetails = {
    haptic: `
      <h3>Haptic Robot Control</h3>
      <p>Teleoperation of a UR3e robot using <strong>MPEG Haptics</strong> and <strong>Geomagic Touch</strong>.</p>
      <img src="images/projects/hapticrobot.png" alt="Haptic Robot Control">
      <p>Real-time bidirectional control with force feedback and low latency.</p>
    `,
    autonomous: `
      <h3>Autonomous Robot</h3>
      <p>Autonomous mobile robot with manual, avoidance and color tracking modes.</p>
      <img src="images/projects/robotros2.png" alt="Autonomous Robot">
    `,
    sensor: `
      <h3>Sensor Network</h3>
      <p>CAN bus network of STM32 nodes communicating with a PC host.</p>
      <img src="images/projects/reseaucapteur.png" alt="Sensor Network">
    `,
    "3d": `
      <h3>3D Design & Printing</h3>
      <p>Prototyping using <strong>Fusion 360</strong> and <strong>Cura</strong>.</p>
      <img src="images/projects/3dprinting.jpg" alt="3D Printing">
    `
  };

  // Clic sur un projet
  projectItems.forEach(item => {
    item.addEventListener("click", () => {
      const key = item.dataset.project;

      // Efface un éventuel détail ouvert
      container.innerHTML = "";

      // Crée le bloc de détails
      const detailDiv = document.createElement("div");
      detailDiv.classList.add("project-details");
      detailDiv.innerHTML = `
        <span class="close-detail">&times;</span>
        ${projectDetails[key]}
      `;
      container.appendChild(detailDiv);

      // Animation douce
      detailDiv.scrollIntoView({ behavior: "smooth" });

      // Fermeture
      detailDiv.querySelector(".close-detail").addEventListener("click", () => {
        detailDiv.remove();
      });
    });
  });
});
