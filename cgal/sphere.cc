	PolySet *p = new PolySet(3,true);
		g = p;
		if (this->r1 > 0 && !isinf(this->r1)) {
			struct ring_s {
				point2d *points;
				double z;
			};

			int fragments = Calc::get_fragments_from_r(r1, fn, fs, fa);
			int rings = (fragments+1)/2;
// Uncomment the following three lines to enable experimental sphere tesselation
//		if (rings % 2 == 0) rings++; // To ensure that the middle ring is at phi == 0 degrees

			ring_s *ring = new ring_s[rings];

//		double offset = 0.5 * ((fragments / 2) % 2);
			for (int i = 0; i < rings; i++) {
//			double phi = (M_PI * (i + offset)) / (fragments/2);
				double phi = (M_PI * (i + 0.5)) / rings;
				double r = r1 * sin(phi);
				ring[i].z = r1 * cos(phi);
				ring[i].points = new point2d[fragments];
				generate_circle(ring[i].points, r, fragments);
			}

			p->append_poly();
			for (int i = 0; i < fragments; i++)
				p->append_vertex(ring[0].points[i].x, ring[0].points[i].y, ring[0].z);

			for (int i = 0; i < rings-1; i++) {
				ring_s *r1 = &ring[i];
				ring_s *r2 = &ring[i+1];
				int r1i = 0, r2i = 0;
				while (r1i < fragments || r2i < fragments)
				{
					if (r1i >= fragments)
						goto sphere_next_r2;
					if (r2i >= fragments)
						goto sphere_next_r1;
					if ((double)r1i / fragments <
							(double)r2i / fragments)
					{
					sphere_next_r1:
						p->append_poly();
						int r1j = (r1i+1) % fragments;
						p->insert_vertex(r1->points[r1i].x, r1->points[r1i].y, r1->z);
						p->insert_vertex(r1->points[r1j].x, r1->points[r1j].y, r1->z);
						p->insert_vertex(r2->points[r2i % fragments].x, r2->points[r2i % fragments].y, r2->z);
						r1i++;
					} else {
					sphere_next_r2:
						p->append_poly();
						int r2j = (r2i+1) % fragments;
						p->append_vertex(r2->points[r2i].x, r2->points[r2i].y, r2->z);
						p->append_vertex(r2->points[r2j].x, r2->points[r2j].y, r2->z);
						p->append_vertex(r1->points[r1i % fragments].x, r1->points[r1i % fragments].y, r1->z);
						r2i++;
					}
				}
			}

			p->append_poly();
			for (int i = 0; i < fragments; i++)
				p->insert_vertex(ring[rings-1].points[i].x, 
												 ring[rings-1].points[i].y, 
												 ring[rings-1].z);

			for (int i = 0; i < rings; i++) {
				delete[] ring[i].points;
			}
			delete[] ring;
		}
	}
