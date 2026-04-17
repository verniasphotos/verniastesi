import sys
import re

file_path = "/Users/vernias/Desktop/verniastesi/Report/main.tex"

with open(file_path, "r", encoding="utf-8") as f:
    text = f.read()

new_biblio = r"""\begin{thebibliography}{99}

\vspace{\baselineskip}

\bibitem{Huang2022}
Jialing Huang, Cheng-Xiang Wang*, Yingzhuo Sun, Jie Huang, Fu-Chun. \textit{A Novel Ray Tracing Based 6G RIS Wireless Channel Model and RIS Deployment Studies in Indoor Scenarios}. 
\url{https://ieeexplore.ieee.org/document/9977575}

\bibitem{Zhang2022}
Zijian Zhang, Changhao Liu, Linglong Dai, Fan Yang, H. Vincent Poor, Xibi Chen, Robert Schober. \textit{Active RIS vs. Passive RIS Which Will Prevail in 6G}. 
\url{https://ieeexplore.ieee.org/document/9998527}

\bibitem{Vairam2023}
T. Vairam, Vishal K, Kavin T, Sengathirsoorian E. T, Murugavel E. \textit{An Empirical Evaluation of gRPC and REST Communication Patterns in Microservice Architectures}, Department of Information Technology PSG College of Technology Coimbatore, India. 
\url{https://ieeexplore.ieee.org/document/11448051}

\bibitem{Muller2024}
David Muller, Kevin Weinberger, Raphael Dyrska, Aydin Sezgin, Martin Mönnigmann. \textit{Assessing EKF-based Orientation Uncertainties and its Impact on the Channels of UAV-mounted RIS}, Automatic Control and Systems Theory, Ruhr-Universität Bochum, Germany, Institute of Digital Communication Systems, Ruhr-Universität Bochum, Germany. 
\url{https://ieeexplore.ieee.org/document/10757906}

\bibitem{Ye2023}
Zi Ye, Faryal Junaid, Rickard Nilsson, Jaap van de Beek. \textit{Autonomous Single Antenna Receiver Localization and Tracking with RIS and EKF}, Department of Computer Science, Electrical and Space Engineering, Luleå University of Technology, Sweden. 
\url{https://ieeexplore.ieee.org/document/10188365}

\bibitem{Wang2024}
Kaining Wang, Bo Yang, Yusheng Lei, Zhiwen Yu, Xuelin Cao, George C. Alexandropoulos, Marco Di Renzo, Chau Yuen. \textit{Dynamical ON-OFF Control with Trajectory Prediction for Multi-RIS Wireless Networks}. 
\url{https://ieeexplore.ieee.org/document/11432189}

\bibitem{Khan2024}
Wali Ullah Khan, Chandan Kumar Sheemar, Syed Tariq Shah, Symeon Chatzinotas. \textit{Energy Efficiency Optimization for CR-Enabled Integrated Terrestrial and NTNs with BD-RIS}. 
\url{https://ieeexplore.ieee.org/document/11275408}

\bibitem{Azizi2024}
Arman Azizi, Mustafa A. Kishk, Arman Farhang. \textit{Exploring the Impact of HAPS-RIS on UAV-Based Networks: a Novel Network Architecture}. 
\url{https://ieeexplore.ieee.org/document/11274933}

\bibitem{Kolakowski2024}
Robert Kołakowski, Thierry Lejkin, Victorien Romain. \textit{Hybrid SDN-Based Data-Centric Mobile Core Network with Content-Based Routing}. 
\url{https://ieeexplore.ieee.org/document/11314080}

\bibitem{Wei2024}
Tongyi Wei, Beixiong Zheng, Weizhi Chen, Kun Tang, Wenjie Feng, Wenquan Che, Quan Xue. \textit{Machine Learning-Enhanced Beamforming in RIS-Assisted 6G SAGIN IoT: Principles, Applications, and Management}. 
\url{https://ieeexplore.ieee.org/document/10965440}

\bibitem{Gebretsadik2024}
Atli Lemma Gebretsadik, Raju Malleboina, Debdeep Sarkar. \textit{On the Path Loss Characteristics of Sub-6 GHz 6G Indoor Channels Enabled With Coded Metasurface Based Anomalous Reflectors}, Department of Electrical Communication Engineering, Indian Institute of Science Bangalore, Karnataka, India. 
\url{https://ieeexplore.ieee.org/document/10677121}

\bibitem{Lien2024}
Shao-Yu Lien, Chih-Cheng Tseng, Wei-Cheng Hung, Cheng-You Tsai, Ting-Yu Liu, Der-Jiunn Deng, Yuan-Chun Lin, Shih-Cheng Lin, Chia-Chan Chang, Sheng-Fuh Chang. \textit{Open Radio Access Network RIC Empowered Reconfigurable Intelligent Surface: A Physical-Layer Security Perspective}. 
\url{https://ieeexplore.ieee.org/document/10726744}

\bibitem{Yu2024}
Xiaoyi Yu, Ruiliang Song, Yun Liu, Zikai Wang, Haipeng Zhang, Waiho MOW, Pei Xiao. \textit{Research on Grant-free Random Access Based on Predictive Compressed Sensing}. 
\url{https://ieeexplore.ieee.org/document/11163255}

\bibitem{Poddar2024}
Hitesh Poddar, Tomoki Yoshimura, Art Ishii. \textit{Validation of 3GPP TR 38.901 Indoor Hotspot Path Loss Model Based on Measurements Conducted at 6.75, 16.95, 28 and 73 Ghz for 6G and Beyond}, Sharp Laboratories of America, Vancouver WA, USA. 
\url{https://ieeexplore.ieee.org/document/11174821}

\end{thebibliography}"""

pattern = re.compile(r"\\begin\{thebibliography\}.*?\\end\{thebibliography\}", flags=re.DOTALL)
new_text, subs = pattern.subn(lambda m: new_biblio, text)

print(f"Made {subs} replacements")

if subs == 1:
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(new_text)

