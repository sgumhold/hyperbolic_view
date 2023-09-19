#pragma once

#include <deque>
#include <random>
#include <cgv/base/node.h>
#include <cgv/data/data_format.h>
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/math/ftransform.h>
#include <cgv/media/color.h>
#include <cgv/gui/animate.h>
#include <cgv/render/texture.h>
#include <cgv/render/drawable.h>
#include <cgv/render/view.h>
#include <cgv/render/shader_program.h>
#include <cgv/utils/pointer_test.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/event_handler.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/cone_renderer.h>
#include <cgv_gl/point_renderer.h>
#include <cgv_gl/surfel_renderer.h>
#include <cgv_gl/line_renderer.h>
#include <plot/plot2d.h>
#include <cgv/base/find_action.h>
#include <fstream>

#include "lib_begin.h"

class CGV_API hyperbolic_view :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
public:
	//void pick_ray(int x, int y);
	dmat4 DPV;
	cgv::render::view* view_ptr = 0;
protected:
	cgv::render::shader_program point_prog;
	cgv::render::shader_program edge_prog;
	cgv::render::shader_program sphere_prog;
	struct my_cone_render : public cgv::render::cone_renderer
	{
		bool build_shader_program(cgv::render::context& ctx, cgv::render::shader_program& prog, const cgv::render::shader_define_map& defines)
		{
			return prog.build_program(ctx, "hyperbolic_cone.glpr", true, defines);
		}
	} cr;
	//cgv::render::shader_program cone_prog;
	bool use_spheres = true;
	bool use_cones = true;
	float lambda = 0.0f;
	unsigned nr_samples = 5;
	int dimension = 2;
	unsigned strip_length = 5;
	vec3 translation = vec3(0.0f);
	bool simple_translation = true;
	bool skip_first = true;
	bool use_shader = true;
	bool show_disk = false;
	bool show_sphere = true;
	cgv::render::sphere_render_style sphere_style;
	cgv::render::surfel_render_style disk_style;

	std::vector<vec4> P0;
	std::vector<vec3> P, P_edges;
	std::vector<uint32_t> I0, I_edges;
	cgv::render::point_render_style prs;
	cgv::render::sphere_render_style srs;
	cgv::render::cone_render_style crs;
	cgv::render::line_render_style lrs;
	///
	void generate_network(unsigned n = 500, unsigned m = 1000)
	{
		std::default_random_engine e;
		std::normal_distribution<float> d;
		std::uniform_int_distribution<int> d_idx(0,n-1);
		for (unsigned i = 0; i < n; ++i) {
			vec4 p(d(e), d(e), 0.0f, 0.0f);
			float l = p.length();
			if (l > 1e-6)
				p *= (1.0f-exp(-l))/l;
			P0.push_back(p);
		}
		for (unsigned j = 0; j < m; ++j) {
			I0.push_back(d_idx(e));
			I0.push_back(d_idx(e));
		}
	}
	bool read_network(const std::string& fn_points, const std::string& fn_edges, int dim = 2)
	{
		return read_points(fn_points, dim) && read_edges(fn_edges);
	}
	bool read_points(const std::string& fn, int dim = 2)
	{
		std::ifstream is(fn);
		if (is.fail())
			return false;
		dimension = dim;
		P0.clear();
		float x1, x2, x3, x4 = 0.0f;
		while (true) {
			is >> x2 >> x3;
			if (dim == 3)
				is >> x4;
			if (is.fail())
				break;
			x1 = sqrt(x2 * x2 + x3 * x3 + x4*x4 + 1.0f);
			P0.push_back(vec4(x1, x2, x3, x4));
		}
		return true;
	}
	bool read_edges(const std::string& fn)
	{
		std::ifstream is(fn);
		if (is.fail())
			return false;
		I0.clear();
		int i1, i2;
		while (true) {
			is >> i1 >> i2;
			if (is.fail())
				break;
			I0.push_back(i1-1);
			I0.push_back(i2-1);
		}
		return true;
	}
	vec4 hyperbolic_mixture(const vec4& x, float t, const vec4& y) const
	{
		float p = x[0] * y[0] - (x[1] * y[1] + x[2] * y[2]+ x[3] * y[3]);
		vec4 v = (y - p * x) / sqrt(p * p - 1);
		t *= acosh(p);
		return cosh(t) * x + sinh(t) * v;
	}
	vec3 project(const vec4& p) const
	{
		float x2 = p[1], x3 = p[2], x4 = p[3];
		float x1 = sqrt(x2 * x2 + x3 * x3 + x4 * x4 + 1);
		float z1 = x2 / (x1 + 1.0f);
		float z2 = x3 / (x1 + 1.0f);
		float z3 = x4 / (x1 + 1.0f);
		vec4 q(0.0f, z1, z2, z3);
		vec4 m = (1.0f - lambda)* p + lambda * q;
		if (skip_first)
			return vec3(m[1], m[2], m[3]);
		else
			return vec3(m[0], m[1], m[2]);
	}
	vec4 transform(const vec4& x) const
	{
		vec3 xb = translation;
		float c = dot(xb, xb);
		mat3 M = cgv::math::identity3<float>();
		if (fabs(c) > 1e-6f)
			M += (sqrt(c + 1) - 1) / c * mat3(xb, xb);
		mat4 T;
		float x0 = sqrt(dot(translation, translation) + 1.0f);
		T.set_col(0, vec4(x0, -translation[0], -translation[1], -translation[2]));
		T.set_col(1, vec4(-translation[0], M(0, 0), M(1, 0), M(2, 0)));
		T.set_col(2, vec4(-translation[1], M(0, 1), M(1, 1), M(2, 1)));
		T.set_col(3, vec4(-translation[2], M(0, 2), M(1, 2), M(2, 2)));
		vec4 y;
		if (simple_translation)
			y = x - vec4(0.0f,translation[0],translation[1],translation[2]);
		else
			y = T * x;
		y[0] = sqrt(y[1] * y[1] + y[2] * y[2] + y[3] * y[3] + 1);
		return y;
	}
	void compute_points()
	{
		if (use_shader)
			return;
		P.resize(P0.size());
		for (size_t i = 0; i < P0.size(); ++i)
			P[i] = project(transform(P0[i]));
	}
	void compute_edges()
	{
		if (use_shader)
			return;
		P_edges.resize((nr_samples + 1) * I0.size());
		I_edges.resize(nr_samples * I0.size());
		for (size_t i = 0; i < I0.size(); i += 2) {
			vec4 p0 = transform(P0[I0[i]]);
			vec4 p1 = transform(P0[I0[i + 1]]);
			float s = 1.0f / nr_samples;
			for (unsigned j = 0; j <= nr_samples; ++j) {
				float l = s * j;
				vec4 p = hyperbolic_mixture(p0, l, p1);
				uint32_t idx = uint32_t((nr_samples + 1) * i + j);
				P_edges[idx] = project(p);
				if (j < nr_samples) {
					I_edges[nr_samples * i + 2 * j] = idx;
					I_edges[nr_samples * i + 2 * j + 1] = idx + 1;
				}
			}
		}
	}
public:
	/// construct with initialized mesh
	hyperbolic_view() : cgv::base::node("hyperbolic view")
	{
		//generate_network();
		read_network("D:/data/graph/fb_hydra_adj.txt", "D:/data/graph/facebook_edgelist.txt");
		//read_network("D:/data/graph/fb_hydra_adj_3d.txt", "D:/data/graph/facebook_edgelist.txt", 3);
		//read_network("D:/data/graph/fb_hydra_adj.txt", "D:/data/graph/facebook_edgelist_subsample.txt");
		compute_points();
		compute_edges();
		srs.radius = 0.02f;
		crs.radius = 0.007f;
		crs.show_caps = true;
		prs.point_size = 10;
		prs.halo_color_strength = 1.0f;
		prs.halo_color = rgba(0, 0, 0, 1.0f);
		prs.halo_width_in_pixel = 0;
		prs.blend_points = false;
		prs.blend_width_in_pixel = 0;
		srs.surface_color = rgba(1, 0, 0, 1);
		crs.surface_color = lrs.default_color = rgba(0.3f, 0.3f, 0.3f, 1.0f);
		crs.rounded_caps = true;
		lrs.default_line_width = 3;
		lrs.halo_color_strength = 1.0f;
		//lrs.halo_width_in_pixel = 1.0f;
		lrs.halo_color = rgba(0,0,1,1.0f);
		disk_style.measure_point_size_in_pixel = false;
		disk_style.point_size = 2.0f;
		disk_style.percentual_halo_width = 3.0f;
		disk_style.surface_color = rgb(1.0f, 1.0f, 0.5f);
		disk_style.halo_color_strength = 1.0f;
		disk_style.halo_color = rgba(0.8f, 0.8f, 0.0f, 1.0f);
		disk_style.illumination_mode = cgv::render::IM_OFF;

		sphere_style.culling_mode = cgv::render::CM_FRONTFACE;
		sphere_style.radius = 1.0f;
		sphere_style.surface_color = rgb(1.0f, 1.0f, 0.5f);
		sphere_style.percentual_halo_width = 3.0f;
		sphere_style.halo_color_strength = 1.0f;
		sphere_style.halo_color = rgba(0.8f, 0.8f, 0.0f, 1.0f);
	}
	/// update
	void on_set(void* member_ptr)
	{
		cgv::utils::pointer_test pt(member_ptr);
		if (pt.member_of(crs.surface_color)) {
			lrs.default_color = crs.surface_color;
			for (int i=0; i<4; ++i)
				update_member(&lrs.default_color[i]);
		}
		if (pt.one_of(skip_first, simple_translation, use_shader, lambda) || pt.member_of(translation)) {
			compute_points();
			compute_edges();
		}
		if (pt.one_of(nr_samples, use_shader))
			compute_edges();
		
		update_member(member_ptr);
		post_redraw();
	}
	/// overload methods of drawable
	std::string get_type_name() const { return "hyperbolic_view"; }
	///
	bool init(cgv::render::context& ctx)
	{
		ctx.set_bg_clr_idx(4);
		cgv::render::ref_point_renderer(ctx, 1);
		cgv::render::ref_line_renderer(ctx, 1);
		//cgv::render::ref_cone_renderer(ctx, 1);
		cr.init(ctx);
		cgv::render::ref_sphere_renderer(ctx, 1);
		cgv::render::ref_surfel_renderer(ctx, 1);
		
		if (!point_prog.build_program(ctx, "hyperbolic_point.glpr"))
			return false;
		if (!edge_prog.build_program(ctx, "hyperbolic_edge.glpr"))
			return false;
		if (!sphere_prog.build_program(ctx, "hyperbolic_sphere.glpr"))
			return false;
//		if (!cone_prog.build_program(ctx, "hyperbolic_cone.glpr"))
//			return false;
		return true;
	}
	///
	void init_frame(cgv::render::context& ctx)
	{
		if (view_ptr == 0)
			view_ptr = find_view_as_node();
		//pr.set_y_view_angle(float(view_ptr->get_y_view_angle()));
	}
	///
	void draw(cgv::render::context& ctx)
	{
//		glDisable(GL_DEPTH_TEST);
//		glEnable(GL_BLEND);
//		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		if (P0.size() > 0) {
			ctx.set_color(srs.surface_color);
			if (use_spheres) {
				auto& sr = cgv::render::ref_sphere_renderer(ctx);
				if (use_shader && sphere_prog.is_linked()) {
					sr.set_prog(sphere_prog);
					sphere_prog.set_uniform(ctx, "dimension", dimension);
					sphere_prog.set_uniform(ctx, "hyperbolic_translation", translation);
					sphere_prog.set_uniform(ctx, "hyperbolic_lambda", lambda);
					sphere_prog.set_uniform(ctx, "simple_translation", simple_translation);
					sphere_prog.set_uniform(ctx, "skip_first", skip_first);
					sr.set_render_style(srs);
					sr.set_y_view_angle((float)view_ptr->get_y_view_angle());
					sr.set_position_array(ctx, P0);
					sr.render(ctx, 0, P0.size());
				}
				else {
					sr.set_render_style(srs);
					sr.set_y_view_angle((float)view_ptr->get_y_view_angle());
					sr.set_position_array(ctx, P);
					sr.render(ctx, 0, P.size());
				}
			}
			else {
				auto& pr = cgv::render::ref_point_renderer(ctx);
				if (use_shader && point_prog.is_linked()) {
					pr.set_prog(point_prog);
					point_prog.set_uniform(ctx, "dimension", dimension);
					point_prog.set_uniform(ctx, "hyperbolic_translation", translation);
					point_prog.set_uniform(ctx, "hyperbolic_lambda", lambda);
					point_prog.set_uniform(ctx, "simple_translation", simple_translation);
					point_prog.set_uniform(ctx, "skip_first", skip_first);
					pr.set_render_style(prs);
					pr.set_y_view_angle((float)view_ptr->get_y_view_angle());
					pr.set_position_array(ctx, P0);
					pr.render(ctx, 0, P0.size());
				}
				else {
					pr.set_render_style(prs);
					pr.set_y_view_angle((float)view_ptr->get_y_view_angle());
					pr.set_position_array(ctx, P);
					pr.render(ctx, 0, P.size());
				}
			}
		}
		if (I0.size() > 0) {
			if (use_cones) {
//				auto& cr = cgv::render::ref_cone_renderer(ctx);
				if (use_shader) {
//					cr.set_prog(cone_prog);
					//cone_prog.set_uniform(ctx, "strip_length", int(strip_length));
					cr.set_render_style(crs);
					cr.set_position_array(ctx, P0);
					cr.set_indices(ctx, I0);
					if (cr.validate_and_enable(ctx)) {
						auto& cone_prog = cr.ref_prog();
						cone_prog.set_uniform(ctx, "dimension", dimension);
						cone_prog.set_uniform(ctx, "nr_samples", int(nr_samples * strip_length));
						cone_prog.set_uniform(ctx, "hyperbolic_translation", translation);
						cone_prog.set_uniform(ctx, "hyperbolic_lambda", lambda);
						cone_prog.set_uniform(ctx, "simple_translation", simple_translation);
						cone_prog.set_uniform(ctx, "skip_first", skip_first);
						cr.draw_impl_instanced(ctx, cgv::render::PT_LINES, 0, I0.size(), nr_samples*strip_length);
						cr.disable(ctx);
					}
				}
				else {
					cr.set_render_style(crs);
					cr.set_position_array(ctx, P_edges);
					cr.set_indices(ctx, I_edges);
					cr.render(ctx, 0, I_edges.size());
				}
			}
			else {
				auto& lr = cgv::render::ref_line_renderer(ctx);
				if (use_shader && edge_prog.is_linked()) {
					lr.set_prog(edge_prog);
					edge_prog.set_uniform(ctx, "dimension", dimension);
					edge_prog.set_uniform(ctx, "strip_length", int(strip_length));
					edge_prog.set_uniform(ctx, "nr_samples", int(nr_samples));
					edge_prog.set_uniform(ctx, "hyperbolic_translation", translation);
					edge_prog.set_uniform(ctx, "hyperbolic_lambda", lambda);
					edge_prog.set_uniform(ctx, "simple_translation", simple_translation);
					edge_prog.set_uniform(ctx, "skip_first", skip_first);
					lr.set_render_style(lrs);
					lr.set_position_array(ctx, P0);
					lr.set_indices(ctx, I0);
					if (lr.validate_and_enable(ctx)) {
						lr.draw_impl_instanced(ctx, cgv::render::PT_LINES, 0, I0.size(), nr_samples);
						lr.disable(ctx);
					}
					lr.render(ctx, 0, I0.size());
				} else {
					lr.set_render_style(lrs);
					lr.set_position_array(ctx, P_edges);
					lr.set_indices(ctx, I_edges);
					lr.render(ctx, 0, I_edges.size());
				}
			}
		}

		if (show_disk) {
			auto& sr = cgv::render::ref_surfel_renderer(ctx);
			sr.set_render_style(disk_style);
			sr.set_reference_point_size(1.0f);
			vec3 p(0.0f);
			vec3 n(0.0f, 0.0f, 1.0f);
			sr.set_position(ctx, p);
			sr.set_normal(ctx, n);
			sr.render(ctx, 0, 1);
		}
		if (show_sphere) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(sphere_style);
			vec3 p(0.0f);
			sr.set_position(ctx, p);
			sr.render(ctx, 0, 1);
		}
		//		glDisable(GL_BLEND);
//		glEnable(GL_DEPTH_TEST);
	}
	///
	void clear(cgv::render::context& ctx)
	{
		cgv::render::ref_point_renderer(ctx, -1);
		//cgv::render::ref_cone_renderer(ctx, -1);
		cr.clear(ctx);
		cgv::render::ref_line_renderer(ctx, -1);
		cgv::render::ref_sphere_renderer(ctx, -1);
		cgv::render::ref_surfel_renderer(ctx, -1);
		point_prog.destruct(ctx);
		edge_prog.destruct(ctx);
//		cone_prog.destruct(ctx);
		sphere_prog.destruct(ctx);
	}
	/// overload and implement this method to handle events
	bool handle(cgv::gui::event& e)
	{
		if (e.get_kind() == cgv::gui::EID_KEY) {
			auto& ke = reinterpret_cast<cgv::gui::key_event&>(e);
			if (ke.get_action() != cgv::gui::KA_RELEASE) {
				switch (ke.get_char()) {
				case '-':
					if (nr_samples > 1) {
						--nr_samples;
						on_set(&nr_samples);
					}
					return true;
				case '+':
					++nr_samples;
					on_set(&nr_samples);
					return true;
				case '_':
					if (strip_length > 1) {
						--strip_length;
						on_set(&strip_length);
					}
					return true;
				case '*':
					++strip_length;
					on_set(&strip_length);
					return true;
				}
				switch (ke.get_key()) {
				case cgv::gui::KEY_Left:
					translation[0] -= 0.1f;
					on_set(&translation[0]);
					return true;
				case cgv::gui::KEY_Right:
					translation[0] += 0.1f;
					on_set(&translation[0]);
					return true;
				case cgv::gui::KEY_Down:
					translation[1] -= 0.1f;
					on_set(&translation[1]);
					return true;
				case cgv::gui::KEY_Up:
					translation[1] += 0.1f;
					on_set(&translation[1]);
					return true;
				case cgv::gui::KEY_Page_Down:
					translation[2] -= 0.1f;
					on_set(&translation[2]);
					return true;
				case cgv::gui::KEY_Page_Up:
					translation[2] += 0.1f;
					on_set(&translation[2]);
					return true;
				case cgv::gui::KEY_Home:
					cgv::gui::animate_with_linear_blend(translation[0], 0.0f, 0.2f*translation.length())->set_base_ptr(this);
					cgv::gui::animate_with_linear_blend(translation[1], 0.0f, 0.2f*translation.length())->set_base_ptr(this);
					cgv::gui::animate_with_linear_blend(translation[2], 0.0f, 0.2f*translation.length())->set_base_ptr(this);
					//translation[0] = 0.0f;
					//on_set(&translation[0]);
					//translation[1] = 0.0f;
					//on_set(&translation[1]);
					//translation[2] = 0.0f;
					//on_set(&translation[2]);
					return true;
				case '0': 
					cgv::gui::animate_with_linear_blend(lambda, 0.0f, lambda+0.1)->set_base_ptr(this);
					return true;
				case '5':
					cgv::gui::animate_with_linear_blend(lambda, 0.5f, fabs(lambda - 0.5))->set_base_ptr(this);
					return true;
				case '1':
					cgv::gui::animate_with_linear_blend(lambda, 1.0f, fabs(lambda - 1.0))->set_base_ptr(this);
					return true;
				}
			}
		}
		return false;
	}
	/// overload to stream help information to the given output stream
	void stream_help(std::ostream& os)
	{
	}
	void stream_stats(std::ostream& os)
	{
	}
	///
	void create_gui()
	{
		add_member_control(this, "show disk", show_disk, "toggle", "shortcut='D'");
		if (begin_tree_node("disk style", disk_style)) {
			align("\a");
			add_gui("style", disk_style);
			align("\b");
			end_tree_node(disk_style);
		}
		add_member_control(this, "show sphere", show_sphere, "toggle");
		if (begin_tree_node("sphere style", sphere_style)) {
			align("\a");
			add_gui("style", sphere_style);
			align("\b");
			end_tree_node(sphere_style);
		}
		add_member_control(this, "use shader", use_shader, "toggle", "shortcut='X'");
		add_member_control(this, "use spheres", use_spheres, "check", "shortcut='S'");
		add_member_control(this, "use cones", use_cones, "check", "shortcut='C'");
		add_member_control(this, "skip first", skip_first, "check", "shortcut='F'");
		add_member_control(this, "use simple_translation", simple_translation, "check", "shortcut='T'");
		add_gui("translation", translation, "", "options='min=-10;max=10;ticks=true'");
		add_member_control(this, "point color", srs.surface_color);
		add_member_control(this, "line color", crs.surface_color);
		add_member_control(this, "lambda", lambda, "value_slider", "min=0;max=1;ticks=true");
		add_member_control(this, "dimension", dimension, "value_slider", "min=2;max=3;ticks=true");
		add_member_control(this, "nr_samples", nr_samples, "value_slider", "min=1;max=50;log=true;ticks=true");
		add_member_control(this, "strip_length", strip_length, "value_slider", "min=1;max=10;log=true;ticks=true");
		if (begin_tree_node("point style", prs)) {
			align("\a");
			add_gui("style", prs);
			align("\b");
			end_tree_node(prs);
		}
		if (begin_tree_node("sphere style", srs)) {
			align("\a");
			add_gui("style", srs);
			align("\b");
			end_tree_node(srs);
		}
		if (begin_tree_node("line style", lrs)) {
			align("\a");
			add_gui("style", lrs);
			align("\b");
			end_tree_node(lrs);
		}
		if (begin_tree_node("cone style", crs)) {
			align("\a");
			add_gui("style", crs);
			align("\b");
			end_tree_node(crs);
		}
	}
};

#include <cgv/config/lib_end.h>


#include <cgv/base/register.h>

#include "lib_begin.h"

extern CGV_API cgv::base::object_registration<hyperbolic_view> hyperbolic_view_reg("register hyperbolic view");
