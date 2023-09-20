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
#include <cgv/utils/file.h>
#include <cgv/utils/advanced_scan.h>
#include <cgv/base/register.h>
#include <cgv/render/texture.h>
#include <cgv/render/drawable.h>
#include <cgv/render/view.h>
#include <cgv/render/shader_program.h>
#include <cgv/utils/pointer_test.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/mouse_event.h>
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
	public cgv::base::argument_handler,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
public:
	//void pick_ray(int x, int y);
	dmat4 DPV;
	cgv::render::view* view_ptr = 0;
	std::string file_name;
	std::string points_file_name;
	std::string edges_file_name;
	enum NodeStyle {
		NS_HIDE,
		NS_POINT,
		NS_SPHERE
	} node_style = NS_SPHERE;
	enum LinkStyle {
		LS_HIDE,
		LS_LINE,
		LS_CONE
	} link_style = LS_CONE;
protected:
	std::string dnd_text;
	int dnd_x, dnd_y;
	void handle_args(std::vector<std::string>& args)
	{
		if (args.size() < 3) {
			std::cerr << "usage:" << cgv::utils::file::get_file_name(args[0])
				<< " <node file> <link file>" << std::endl;
			return;
		}
		read_points_or_edges(args[1]);
		read_points_or_edges(args[2]);
	}
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
	float lambda = 1.0f;
	unsigned nr_samples = 12;
	int dimension = 2;
	vec3 translation = vec3(0.0f);
	mat4 T, iT;
	bool simple_translation = false;
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
	void configure_dimension(int dim)
	{
		dimension = dim;
		if (dim == 2) {
			show_disk = true;
			show_sphere = false;
			translation[2] = 0;
			on_set(&translation[2]);
		}
		else {
			show_disk = false;
			show_sphere = true;
		}
		auto cp = find_control(translation[2]);
		if (cp)
			cp->set("active", dimension == 3);
		on_set(&show_disk);
		on_set(&show_sphere);
		on_set(&dimension);
	}
	void generate_network(unsigned n = 500, unsigned m = 2000, int dim = 3)
	{
		std::default_random_engine e;
		std::normal_distribution<float> d;
		std::uniform_int_distribution<int> d_idx(0,n-1);
		std::uniform_real_distribution<float> d_flt(0.0f,1.0f);
		for (unsigned i = 0; i < n; ++i) {
			vec4 p(0.0f, d(e), d(e), dim == 2 ? 0.0f : d(e));
			p[0] = sqrt(p[1] * p[1] + p[2] * p[2] + p[3]*p[3] + 1);
			P0.push_back(p);
		}
		configure_dimension(dim);
		std::set<std::pair<int, int>> edges;
		while (I0.size() < 2*m) {
			int i = d_idx(e), j = d_idx(e);
			if (i == j)
				continue;
			std::pair<int, int> edge = { i,j };
			if (edges.find(edge) != edges.end())
				continue;
			float l = (P0[j] - P0[i]).length();
			float prob = exp(-10*l);
			if (d_flt(e) < prob) {
				I0.push_back(i);
				I0.push_back(j);
				edges.insert(edge);
			}
		}
	}
	bool read_network(const std::string& fn_points, const std::string& fn_edges, int dim = -1)
	{
		return read_points(fn_points, dim) && read_edges(fn_edges);
	}
	bool read_file(const std::string& fn, std::vector<dvec3>& data, int& dim) const
	{
		std::string content;
		if (!cgv::utils::file::read(fn, content, true))
			return false;
		std::vector<cgv::utils::line> lines;
		cgv::utils::split_to_lines(content, lines);
		if (lines.size() < 2)
			return false;
		bool is_csv = false;
		if (dim == -1) {
			std::vector<cgv::utils::token> toks;
			cgv::utils::split_to_tokens(lines[1], toks, "", false, "\"", "\"", " ;\t\n");
			if (toks.size() >= 2) {
				if (*toks.front().end == '"')
					is_csv = true;
				dim = int(toks.size());
				if (is_csv)
					--dim;
				if (dim > 3)
					dim = 3;
			}
		}
		if (dim == -1)
			return false;
		unsigned li = is_csv ? 1 : 0;
		unsigned i0 = li;
		unsigned n = is_csv ? dim + 1 : dim;
		for (; li < lines.size(); ++li) {
			cgv::utils::line l = lines[li];
			std::vector<cgv::utils::token> toks;
			cgv::utils::split_to_tokens(l, toks, "", false, "\"", "\"", " ;\t\n");
			if (toks.size() < n)
				continue;
			dvec3 v(0.0);
			unsigned i = i0;
			for (; i < n; ++i) {
				std::string tok = cgv::utils::to_string(toks[i]);
				cgv::utils::replace(tok, ',', '.');
				if (!cgv::utils::is_double(tok, v[i - i0]))
					break;
			}
			if (i < n)
				continue;
			data.push_back(v);
		}
		return true;
	}
	void extract_points(std::vector<dvec3>& data, int dim)
	{
		configure_dimension(dim);
		P0.resize(data.size());
		for (size_t i = 0; i < data.size(); ++i) {
			float x2 = float(data[i][0]), x3 = float(data[i][1]), x4 = float(data[i][2]);
			float x1 = sqrt(x2 * x2 + x3 * x3 + x4 * x4 + 1.0f);
			P0[i] = vec4(x1, x2, x3, x4);
		}
	}
	void extract_edges(std::vector<dvec3>& data)
	{
		I0.resize(2 * data.size());
		for (size_t i = 0; i < data.size(); ++i) {
			I0[2 * i] = uint32_t(data[i][0]);
			I0[2 * i + 1] = uint32_t(data[i][1]);
		}
	}
	bool read_points(const std::string& fn, int dim = -1)
	{
		std::vector<dvec3> data;
		if (!read_file(fn, data, dim))
			return false;
		extract_points(data, dim);
		return true;
	}
	bool read_edges(const std::string& fn)
	{
		int dim = -1;
		std::vector<dvec3> data;
		if (!read_file(fn, data, dim))
			return false;
		if (dim != 2)
			return false;
		extract_edges(data);
		return true;
	}
	bool read_points_or_edges(const std::string& fn, bool* edges_not_points_ptr = 0)
	{
		int dim = -1;
		std::vector<dvec3> data;
		if (!read_file(fn, data, dim))
			return false;
		bool are_edges = dim == 2;
		if (are_edges) {
			for (const auto& v : data) {
				for (int i = 0; are_edges && i < 2; ++i)
					if (abs(double(int(v[i])) - v[i]) > 1e-16)
						are_edges = false;
				if (!are_edges)
					break;
			}
		}
		if (are_edges)
			extract_edges(data);
		else
			extract_points(data, dim);
		if (edges_not_points_ptr)
			*edges_not_points_ptr = are_edges;
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
		if (dimension == 3)
			return vec3(m[1], m[2], m[3]);
		else
			return vec3(m[1], m[2], m[0]);
	}
	vec4 unproject(const vec3& q) const
	{
		if (lambda < 0.0001f) {
			float x2 = q[0];
			float x3 = q[1];
			float x4 = dimension == 3 ? q[2] : 0.0f;
			float x1 = sqrt(x2 * x2 + x3 * x3 + x4 * x4 + 1);
			return vec4(x1, x2, x3, x4);
		}
		if (lambda > 0.9999f) {
			float c = dot(q, q);
			float x1 = sqrt((c + 1) / (1 - c));
			float x2 = q[0] * (x1 + 1);
			float x3 = q[1] * (x1 + 1);
			float x4 = q[2] * (x1 + 1);
			return vec4(x1, x2, x3, x4);
		}
		return vec4(q[0], 1.0f);
	}
	void compute_translation_matrices()
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
		iT = inv(T);
	}
	vec4 transform(const vec4& x) const
	{
		vec4 y;
		if (simple_translation)
			y = x - vec4(0.0f,translation[0],translation[1],translation[2]);
		else
			y = T * x;
		y[0] = sqrt(y[1] * y[1] + y[2] * y[2] + y[3] * y[3] + 1);
		return y;
	}
	vec4 inv_transform(const vec4& y) const
	{
		vec4 x;
		if (simple_translation)
			x = y + vec4(0.0f,translation[0],translation[1],translation[2]);
		else
			x = iT * y;
		x[0] = sqrt(x[1] * x[1] + x[2] * x[2] + x[3] * x[3] + 1);
		return x;
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
		generate_network();
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
		compute_translation_matrices();
	}
	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("file_name", file_name);
	}
	/// update
	void on_set(void* member_ptr)
	{
		cgv::utils::pointer_test pt(member_ptr);
		if (pt.is(points_file_name))
			read_points(points_file_name);
		if (pt.is(edges_file_name))
			read_edges(edges_file_name);
		if (pt.is(file_name)) {
			bool are_edges;
			if (read_points_or_edges(file_name, &are_edges)) {
				if (are_edges) {
					edges_file_name = file_name;
					update_member(&edges_file_name);
				}
				else {
					points_file_name = file_name;
					update_member(&points_file_name);
				}
			}
		}
		if (pt.member_of(crs.surface_color)) {
			lrs.default_color = crs.surface_color;
			for (int i=0; i<4; ++i)
				update_member(&lrs.default_color[i]);
		}
		if (pt.member_of(translation)) {
			compute_translation_matrices();
		}
		if (pt.one_of(dimension, simple_translation, use_shader, lambda) || pt.member_of(translation)) {
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
	/// set uniforms specific to hyperbolic projection
	void set_hyperbolic_uniforms(cgv::render::context& ctx, cgv::render::shader_program& prog)
	{
		prog.set_uniform(ctx, "dimension", dimension);
		prog.set_uniform(ctx, "hyperbolic_translation", translation);
		prog.set_uniform(ctx, "hyperbolic_lambda", lambda);
		prog.set_uniform(ctx, "simple_translation", simple_translation);
		int ui = prog.get_uniform_location(ctx, "nr_samples");
		if (ui != -1)
			prog.set_uniform(ctx, ui, int(nr_samples));
	}
	///
	void draw(cgv::render::context& ctx)
	{
		DPV = ctx.get_modelview_projection_window_matrix();
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
		if (node_style != NS_HIDE && P0.size() > 0) {
			ctx.set_color(srs.surface_color);
			if (node_style == NS_SPHERE) {
				auto& sr = cgv::render::ref_sphere_renderer(ctx);
				if (use_shader && sphere_prog.is_linked()) {
					sr.set_prog(sphere_prog);
					set_hyperbolic_uniforms(ctx, sphere_prog);
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
					set_hyperbolic_uniforms(ctx, point_prog);
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
		if (link_style != LS_HIDE && I0.size() > 0) {
			if (link_style == LS_CONE) {
				if (use_shader) {
					cr.set_render_style(crs);
					cr.set_position_array(ctx, P0);
					cr.set_indices(ctx, I0);
					if (cr.validate_and_enable(ctx)) {
						auto& cone_prog = cr.ref_prog();
						set_hyperbolic_uniforms(ctx, cone_prog);
						cr.draw_impl_instanced(ctx, cgv::render::PT_LINES, 0, I0.size(), nr_samples);
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
					set_hyperbolic_uniforms(ctx, edge_prog);
					lr.set_render_style(lrs);
					lr.set_position_array(ctx, P0);
					lr.set_indices(ctx, I0);
					if (lr.validate_and_enable(ctx)) {
						lr.draw_impl_instanced(ctx, cgv::render::PT_LINES, 0, I0.size(), std::max(int(1), int(nr_samples / 5)));
						lr.disable(ctx);
					}
				} else {
					lr.set_render_style(lrs);
					lr.set_position_array(ctx, P_edges);
					lr.set_indices(ctx, I_edges);
					lr.render(ctx, 0, I_edges.size());
				}
			}
		}
		if (dnd_text.empty())
			return;
		glDisable(GL_DEPTH_TEST);
		ctx.set_color(rgba(1, 0, 0, 1));
		ctx.push_pixel_coords();
		ctx.set_cursor(dnd_x, dnd_y);
		ctx.output_stream() << dnd_text;
		ctx.output_stream().flush();
		ctx.pop_pixel_coords();
		glEnable(GL_DEPTH_TEST);
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
		if (e.get_kind() == cgv::gui::EID_MOUSE) {
			auto& me = reinterpret_cast<cgv::gui::mouse_event&>(e);
			if ((me.get_flags() & cgv::gui::EF_DND) != 0) {
				switch (me.get_action()) {
				case cgv::gui::MA_ENTER:
					dnd_text = me.get_dnd_text();
				case cgv::gui::MA_DRAG:
					dnd_x = me.get_x();
					dnd_y = get_context()->get_height() - me.get_y() - 1;
					post_redraw();
					return true;
				case cgv::gui::MA_LEAVE:
					dnd_text.clear();
					post_redraw();
					return true;
				case cgv::gui::MA_RELEASE:
					read_points_or_edges(dnd_text);
					post_redraw();
					dnd_text = std::string();
					return true;
				}
			}
			else if (me.get_modifiers() == cgv::gui::EM_ALT && get_context() != 0) {
				auto& ctx = *get_context();
				ctx.make_current();
				vec3 p = ctx.get_model_point(me.get_x(), ctx.get_height() - 1 - me.get_y(), DPV);
				vec4 q = inv_transform(unproject(p));
				switch (me.get_action()) {
				case cgv::gui::MA_PRESS:
					return true;
				case cgv::gui::MA_RELEASE:
					cgv::gui::animate_with_linear_blend(translation[0], q[1], 1, false)->set_base_ptr(this);
					cgv::gui::animate_with_linear_blend(translation[1], q[2], 1, false)->set_base_ptr(this);
					cgv::gui::animate_with_linear_blend(translation[2], q[3], 1, false)->set_base_ptr(this);
					return true;
//				case cgv::gui::MA_MOVE:
					//std::cout << "p = " << p << " -> " << q << std::endl;
//					break;
				}
			}
		}
		else if (e.get_kind() == cgv::gui::EID_KEY) {
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
					return true;
				case 'N' :
					if (node_style == NS_SPHERE)
						node_style = NS_HIDE;
					else
						++(int&)node_style;
					on_set(&node_style);
					return true;
				case 'L' :
					if (link_style == LS_CONE)
						link_style = LS_HIDE;
					else
						++(int&)link_style;
					on_set(&link_style);
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
		add_view("node file", points_file_name);
		add_view("edge file", edges_file_name);
		add_gui("file_name", file_name, "file_name",
			"w=160;open=true;open_title='open node or link file';filter='file (txt,csv):*.txt;*.csv|all files:*.*';");
		add_view("dimension", dimension);
		add_member_control(this, "lambda", lambda, "value_slider", "min=0;max=1;ticks=true");
		add_member_control(this, "use simple_translation", simple_translation, "check", "shortcut='T'");
		add_gui("translation", translation, "", "options='min=-10;max=10;ticks=true'");

		bool show = begin_tree_node("disk", disk_style, false, "options='w=100';align=' '");
		add_member_control(this, "show", show_disk, "toggle", "w=90;shortcut='D'");
		if (show) {
			align("\a");
			add_gui("style", disk_style);
			align("\b");
			end_tree_node(disk_style);
		}
		show = begin_tree_node("sphere", sphere_style, false, "options='w=100';align=' '");
		add_member_control(this, "show sphere", show_sphere, "toggle", "w=90");
		if (show) {
			align("\a");
			add_gui("style", sphere_style);
			align("\b");
			end_tree_node(sphere_style);
		}

		add_member_control(this, "use shader", use_shader, "toggle", "shortcut='X'");
		show = begin_tree_node("nodes", node_style, false, "options='w=60';align=' '");
		add_member_control(this, "", srs.surface_color, "", "w=50", " ");
		add_member_control(this, "", node_style, "dropdown", "w=60;enums='hide;point;sphere'");
		if (show) {
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
			end_tree_node(node_style);
		}
		show = begin_tree_node("links", link_style, false, "options='w=60';align=' '");
		add_member_control(this, "", crs.surface_color, "", "w=50", " ");
		add_member_control(this, "", link_style, "dropdown", "w=60;enums='hide;line;cone'");
		if (show) {
			add_member_control(this, "nr_samples", nr_samples, "value_slider", "min=1;max=50;log=true;ticks=true");
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
	}
};

#include <cgv/config/lib_end.h>


#include <cgv/base/register.h>

#include "lib_begin.h"

extern CGV_API cgv::base::object_registration<hyperbolic_view> hyperbolic_view_reg("register hyperbolic view");

#ifdef REGISTER_SHADER_FILES
#include <cgv/base/register.h>
#include <hyperbolic_view_shader_inc.h>
#endif